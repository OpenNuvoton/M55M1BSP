/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    NAU88C22 configuration tool main program
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "NuMicro.h"
#include "usbd_audio.h"
#include "user_config.h"
#include "i2c_process.h"
#include "codec_config.h"
#include "sysClk_config.h"
#include "adc_converter.h"

// Extern USB Variable Declaration
extern volatile uint32_t g_au32USBPlayRingBuff[];
extern volatile uint32_t g_au32USBRecRingBuff[];
extern volatile uint32_t g_au32I2STxPingPongBuff[];
extern volatile uint32_t g_au32I2SRxPingPongBuff[];
extern volatile uint8_t g_u8I2STxBuffIndex;
extern volatile uint8_t g_u8I2SRxBuffIndex;
extern volatile uint16_t g_u16PlayBack_Read_Ptr;
extern volatile uint16_t g_u16PlayBack_Write_Ptr;
extern volatile uint16_t g_u16PlayBack_Ptrs_Distance;
extern volatile uint16_t g_u16PlayBack_USB_BUFF_UPPER_THRE;
extern volatile uint16_t g_u16PlayBack_USB_BUFF_LOWER_THRE;
extern volatile uint16_t g_u16Record_Read_Ptr;
extern volatile uint16_t g_u16Record_Write_Ptr;

extern volatile uint8_t g_u8AudioRecorderState;
extern volatile uint8_t g_usbd_PlayMute;
extern volatile uint8_t g_usbd_RecMute;
extern volatile uint8_t g_u8USBTxDataLen;

// Extern GUI Variable Declaration
extern volatile uint8_t g_u8HIDCommandProcessing;
extern HIDTRANS_CMD_T g_sHidCmdH2D;  // Host to device
extern HIDTRANS_CMD_T g_sHidCmdD2H;  // Device to host
extern uint8_t g_au8ToI2CDataBuf[];  // Data from PC
extern uint8_t g_au8ToHIDDataBuf[];  // Data to PC
extern uint8_t g_au8USBDmaDataBuf[]; // Data for USBD DMA transfer
extern volatile uint8_t g_u8HIDInDone;
extern volatile uint16_t g_u16I2CRxDataLen;
extern uint8_t g_u8SlaveAddr;
extern volatile uint32_t g_u32I2C_ClockRate;
extern volatile uint16_t g_u16PowerOnDelay;

// Codec Info
extern volatile uint8_t g_u8DeviceID;
extern volatile uint8_t g_u8RegisterAddrH;
extern volatile uint8_t g_u8RegisterAddrL;
extern volatile uint16_t g_u16Data_Len;
extern volatile uint8_t g_u8WriteFail;
extern volatile uint16_t g_u16I2CTxData_Fail_Cnt;

// 7802 Info
extern volatile uint8_t g_u8Qurey_mode;
extern volatile uint8_t g_u8Channel_mode;
extern volatile uint8_t g_u8Sample_Num_H;
extern volatile uint8_t g_u8Sample_Num_L;
extern volatile uint8_t g_u8T1_delay_H;
extern volatile uint8_t g_u8T1_delay_L;
extern volatile uint8_t g_u8T2_delay_H;
extern volatile uint8_t g_u8T2_delay_L;
extern uint8_t g_au8ADCDataBuff[6144]; // max 1024(sample) * 3(byte) * 2 (ch)
extern volatile uint16_t g_u16SampleNum;
extern volatile uint16_t g_u16SampleNum_cnt;
extern volatile uint8_t g_u8read_process;
extern volatile uint8_t g_u8Read_process_stop;
extern volatile uint8_t g_u8_7802TimeOut;
extern volatile uint8_t g_u8_7802Singal_last_ch;

extern volatile uint8_t g_u8USBPlayEn;
extern volatile uint8_t g_u8USBRecEn;
extern volatile uint16_t g_u16PlayBack_MAX_USB_BUFFER_LEN;
extern volatile uint16_t g_u16UAC_MAX_USB_BUFFER_LEN;
extern volatile uint16_t g_u16PlayBack_I2S_BUFF_LEN;
extern volatile uint16_t g_u16UAC_I2S_BUFF_LEN;
extern volatile uint16_t g_u16usbd_PlaySampleRate;
extern volatile uint16_t g_u16usbd_PlaySampleRate_old;
extern volatile uint8_t g_u8UAC_ADAPTIVE;
extern volatile uint32_t g_u32I2S0SampleRate_old;
extern uint32_t g_u32PllClock;

/*----------------------------------------------------------------------------*/
/* Global Variables                                                           */
/*----------------------------------------------------------------------------*/
// FirmWare Version
__attribute__((section(".ARM.__at_0x00108a00"))) const uint32_t g_u32PID = 0x08020100;
volatile int16_t g_i16HIDDataLen;

// I2S parameter
volatile uint32_t g_u32I2S0SampleRate;
volatile uint32_t g_u32I2S0Channel = I2S_TDMCHNUM_2CH;
volatile uint32_t g_u32I2S0Channel_old = I2S_TDMCHNUM_2CH;
volatile uint32_t g_u32I2S0SerialMode;
volatile uint32_t g_u32I2S0Format;
volatile uint32_t g_u32I2S0Mono;
volatile uint32_t g_u32I2S0WordSize;
volatile uint32_t g_u32I2S0FrameSize;
volatile uint8_t g_u8PlaybackDeviceEn = 0;
volatile uint8_t g_u8RecordDeviceEn = 0;
volatile uint8_t g_bUnderFlow = FALSE; // No RingBuffer for reading
volatile uint8_t g_bOverFlow = FALSE;  // No RingBuffer for Writting
uint8_t g_au8Monitor_DataBuff[256]; // max 1024(sample) * 3(byte) * 2 (ch)
volatile uint8_t g8_Board_mode = Board_Normal;

//------------------------------------------------------------------------------
typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t endsrc;
    uint32_t enddest;
    uint32_t offset;
} DMA_DESC_T;

#ifdef __ICCARM__
    #pragma data_alignment=32
    DMA_DESC_T DMA_TXDESC[2];
    DMA_DESC_T DMA_RXDESC[2];
#else
    DMA_DESC_T DMA_TXDESC[2] __attribute__((aligned(32)));
    DMA_DESC_T DMA_RXDESC[2] __attribute__((aligned(32)));
#endif

volatile uint8_t u8DipSwitch;

/*----------------------------------------------------------------------------*/
/* Functions declaration                                                      */
/*----------------------------------------------------------------------------*/
void AudioDeviceCtrl(void);
void PC_ProcHIDCmd(void);
void Flash_Init(void);
void Flash_Memory(void);
void PDMA_Init(void);
void HSUSBD_Init(void);
void I2C_Init(void);
void I2S_Init(void);
void Systick_Init(void);
void I2S_TX_Start(void);
void I2S_TX_Stop(void);
void I2S_RX_Start(void);
void I2S_RX_Stop(void);
void HSUSBD_EnableEPTrans(uint32_t epnum, uint8_t *buf, uint32_t len);
static uint32_t I2S_open(I2S_T *i2s,
                         uint32_t u32MasterSlave,
                         uint32_t u32SampleRate,
                         uint32_t u32WordWidth,
                         uint32_t u32Channels,
                         uint32_t u32Mono,
                         uint32_t u32DataFormat,
                         uint32_t u32I2S0FrameSize);

//------------------------------------------------------------------------------
void AudioDeviceCtrl(void)
{
    if (g_u8USBPlayEn)
    {
        if (!g_u8PlaybackDeviceEn)
        {
            g_u8PlaybackDeviceEn = 1;
            PlaybackPingpongBuffClear();
            I2S_TX_Start();
        }
    }
    else
    {
        if (g_u8PlaybackDeviceEn)
        {
            g_u8PlaybackDeviceEn = 0;
            I2S_TX_Stop();
        }
    }

    if (g_u8USBRecEn)
    {
        if (!g_u8RecordDeviceEn)
        {
            g_u8RecordDeviceEn = 1;
            RecordPingpongBuffClear();
            I2S_RX_Start();
        }
    }
    else
    {
        if (g_u8RecordDeviceEn)
        {
            g_u8RecordDeviceEn = 0;
            I2S_RX_Stop();
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_INIT();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    PDMA_Init(); // PDMA Initialization Controller

    Flash_Init(); // Flash Initialization Controller

    Systick_Init();

    while (g_u16PowerOnDelay < 2000) {} // Delay 2s then start to init codec

    NAU88C22_CodecMst_Init(48000);

    g8_Board_mode = Board_Normal;

    I2C_Init();

    I2S_Init();

    HSUSBD_Init();

    HSUSBD_CLR_SE0();

    Systick_Init();

    SYS_LockReg();

    /* Set the initial logic value of AUDIO_JKEN as Hi */
    SET_GPIO_PD1();
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT);
    PD1 = 1;

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|              Codec Configuration Tool for NAU88C22               |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code is used to configure NAU88C22 via NuVotonAudioGUI.\n");
    printf("  User needs to install NuVotonAudioGUI on PC.\n");

    while (1)
    {
        PC_ProcHIDCmd();
    }
}

/**
 * @brief       Process HID command between Codec and PC
 * @param       None
 * @return      None
 * @details     This function is used to process HID command between Codec and PC.
 *              1. If command is read, then get data from I2C buffer and send to PC.
 *              2. If command is write, then save data to data flash and send Ack to PC.
 */
void PC_ProcHIDCmd(void)
{
    uint32_t HIRCTrimsts;

    if ((g_u8HIDCommandProcessing == FINISHED) || (g_u8read_process == 1))
    {
        uint8_t *ptr;
        uint16_t i;

        // I2C protocol
        if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BONGIOVI ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DPS_SETTING ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DPS_STRING ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DATA_CONFIG ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DATA_PRESET)
        {
            // respond result to PC
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                uint8_t u8FrameCnt = 0;
                g_i16HIDDataLen = g_u16I2CRxDataLen - I2C_PREFIX_LEN; // The first 4 I2C data are length, command and respond, not real data for HID
                g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                g_sHidCmdD2H.u8CommandType = g_au8ToHIDDataBuf[2];

                do
                {
                    g_sHidCmdD2H.u8DataLen = (g_i16HIDDataLen >= 60) ? 60 : (g_i16HIDDataLen % 60);
                    g_sHidCmdD2H.u8FrameNum = u8FrameCnt;

                    if (g_i16HIDDataLen > 60)
                    {
                        g_sHidCmdD2H.u8FrameNum |= 0x80;
                    }

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                    HSUSBD_MemCopy((ptr + HID_PREFIX_LEN), (g_au8ToHIDDataBuf + I2C_PREFIX_LEN + (u8FrameCnt * 60)), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN));
                    HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                    while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                    g_u8HIDInDone = 0;
                    memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                    g_i16HIDDataLen -= 60;
                    u8FrameCnt++;

                } while (g_i16HIDDataLen > 0);
            }
            else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
            {
                // Save data to data flash first and then send Ack to PC
                if (g_u8ApplyFlag)
                {
                    Flash_Memory();
                    g_u8ApplyFlag = 0;
                }

                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;
                g_sHidCmdD2H.u8FrameNum = 0x00;

                if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON) ||
                        g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DATA_CONFIG ||
                        g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DATA_PRESET)
                {
                    g_sHidCmdD2H.u8DataLen = 0x01;
                }
                else
                {
                    g_sHidCmdD2H.u8DataLen = 0x00;
                    g_sHidCmdD2H.u8CommandType = 0x00;
                }

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
                {
                    ptr[HID_PREFIX_LEN + i] = 0;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                // After sending Ack to PC, reset MCU
                if (g_u8ResetFlag)
                {
                    g_u8ResetFlag = 0;
                    SYS_UnlockReg();
                    SYS_ResetChip(); // Chip reset
                }
            }
        }
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_I2C)
        {
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                uint8_t u8FrameCnt = 0;
                g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                g_sHidCmdD2H.u8CommandType = HIDTRANS_CMD_I2C;
                g_i16HIDDataLen = (g_au8ToHIDDataBuf[1] << 8 | g_au8ToHIDDataBuf[0]) + 1; // Add address

                do
                {
                    g_sHidCmdD2H.u8DataLen = (g_i16HIDDataLen >= 60) ? 60 : (g_i16HIDDataLen % 60);
                    g_sHidCmdD2H.u8FrameNum = u8FrameCnt;

                    if (g_i16HIDDataLen > 60)
                    {
                        g_sHidCmdD2H.u8FrameNum |= 0x80;
                    }

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                    if (u8FrameCnt == 0)
                    {
                        ptr[HID_PREFIX_LEN] = g_u8SlaveAddr + 1;
                        HSUSBD_MemCopy((ptr + HID_PREFIX_LEN + 1), (g_au8ToHIDDataBuf), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN - 1));
                    }
                    else
                    {
                        HSUSBD_MemCopy((ptr + HID_PREFIX_LEN), (g_au8ToHIDDataBuf + I2C_PREFIX_LEN + (u8FrameCnt * 60) - 1), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN));
                    }

                    HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                    while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                    g_u8HIDInDone = 0;
                    memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                    g_i16HIDDataLen -= 60;
                    u8FrameCnt++;

                } while (g_i16HIDDataLen > 0);
            }
            else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;
                g_sHidCmdD2H.u8FrameNum = 0x00;
                g_sHidCmdD2H.u8DataLen = 0x01;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
                {
                    ptr[HID_PREFIX_LEN + i] = 0;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
        }
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_I2S_SETTING)
        {

            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                g_sHidCmdD2H.u8DataLen = 0x06;
                g_sHidCmdD2H.u8FrameNum = 0x00;
                g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                switch (g_u32I2S0SampleRate)
                {
                    case 96000:
                        ptr[HID_PREFIX_LEN + 0] = 0;
                        break;

                    case 48000:
                        ptr[HID_PREFIX_LEN + 0] = 1;
                        break;

                    case 32000:
                        ptr[HID_PREFIX_LEN + 0] = 2;
                        break;

                    case 16000:
                        ptr[HID_PREFIX_LEN + 0] = 3;
                        break;

                    case 8000:
                        ptr[HID_PREFIX_LEN + 0] = 4;
                        break;
                }

                switch (g_u32I2S0Channel)
                {
                    case I2S_TDMCHNUM_2CH:
                        ptr[HID_PREFIX_LEN + 1] = 0;
                        break;

                    case I2S_TDMCHNUM_4CH:
                        ptr[HID_PREFIX_LEN + 1] = 1;
                        break;
                }

                switch (g_u32I2S0SerialMode)
                {
                    case I2S_MODE_SLAVE:
                        ptr[HID_PREFIX_LEN + 2] = 0;
                        break;

                    case I2S_MODE_MASTER:
                        ptr[HID_PREFIX_LEN + 2] = 1;
                        break;
                }

                switch (g_u32I2S0Format)
                {
                    case I2S_FORMAT_I2S:
                        ptr[HID_PREFIX_LEN + 3] = 0;
                        break;

                    case I2S_FORMAT_I2S_MSB:
                        ptr[HID_PREFIX_LEN + 3] = 1;
                        break;

                    case I2S_FORMAT_PCM:
                        ptr[HID_PREFIX_LEN + 3] = 2;
                        break;

                    case I2S_FORMAT_PCM_MSB:
                        ptr[HID_PREFIX_LEN + 3] = 3;
                        break;
                }

                switch (g_u32I2S0WordSize)
                {
                    case I2S_DATABIT_16:
                        ptr[HID_PREFIX_LEN + 4] = 0;
                        break;

                    case I2S_DATABIT_32:
                        ptr[HID_PREFIX_LEN + 4] = 1;
                        break;

                    case I2S_DATABIT_24:
                        ptr[HID_PREFIX_LEN + 4] = 2;
                        break;
                }

                switch (g_u32I2S0FrameSize)
                {
                    case I2S_CHWIDTH_16:
                        ptr[HID_PREFIX_LEN + 5] = 0;
                        break;

                    case I2S_CHWIDTH_32:
                        ptr[HID_PREFIX_LEN + 5] = 1;
                        break;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
            else
            {
                // Save data to data flash first and then send Ack to PC
                if (g_u8ApplyFlag)
                {
                    Flash_Memory();
                    g_u8ApplyFlag = 0;
                }

                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;
                g_sHidCmdD2H.u8DataLen = 0x01;
                g_sHidCmdD2H.u8FrameNum = 0x00;
                g_sHidCmdD2H.u8CommandType = 0x00;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
                {
                    ptr[HID_PREFIX_LEN + i] = 0;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                DelayUs(500);

                if ((g_u32I2S0Channel == I2S_TDMCHNUM_4CH) || (g_u32I2S0Channel_old != g_u32I2S0Channel))
                {
                    SYS_UnlockReg();
                    SYS_ResetChip(); // Chip reset
                }
            }
        }
        else if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_7802_CONVERTER) || (g_u8read_process == 1)) // for new version 7802
        {
            if ((g_sHidCmdH2D.u8Header == HIDTRANS_READ) && (g_u8Read_process_stop == 0))
            {
                uint8_t u8FrameCnt = 0;
                uint16_t i;
                // uint8_t ret;
                g_u16SampleNum = g_u8Sample_Num_H << 8 | g_u8Sample_Num_L;

                if (g_u8Channel_mode == ADC_Single)
                    g_i16HIDDataLen = (g_u8Sample_Num_H << 8 | g_u8Sample_Num_L) * 3; // one Sample number = register ->0x12 (low byte)|0x13(middle byte)|0x14(high byte)
                else if (g_u8Channel_mode == ADC_Dual)
                    g_i16HIDDataLen = (g_u8Sample_Num_H << 8 | g_u8Sample_Num_L) * 3 * 2; // one Sample number = register ->0x12 (low byte)|0x13(middle byte)|0x14(high byte) *2 (ch)

                for (i = 0; i < g_u16SampleNum; i++) // if(ret == 0)
                {
                    if (g_u8Qurey_mode == ADC_Time_Delay)
                    {
                        NAU7802_Time_Delay();
                    }
                    else if (g_u8Qurey_mode == ADC_Polling)
                    {
                        NAU7802_Polling();
                    }

                    if (g_u16SampleNum_cnt == (g_u16SampleNum - 1))
                    {
                        g_u8read_process = 0;
                        g_u16SampleNum_cnt = 0;
                    }
                    else
                    {
                        g_u16SampleNum_cnt++;
                        g_u8read_process = 1;
                    }

                    if ((g_u8read_process == 0) && (g_u8_7802TimeOut == 0))
                    {
                        g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                        g_sHidCmdD2H.u8CommandType = HIDTRANS_CMD_7802_CONVERTER;

                        do
                        {
                            g_sHidCmdD2H.u8DataLen = (g_i16HIDDataLen >= 60) ? 60 : (g_i16HIDDataLen % 60);
                            g_sHidCmdD2H.u8FrameNum = u8FrameCnt;

                            if (g_i16HIDDataLen > 60)
                            {
                                g_sHidCmdD2H.u8FrameNum |= 0x80;
                            }

                            ptr = (uint8_t *)g_au8USBDmaDataBuf;
                            HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                            HSUSBD_MemCopy((ptr + HID_PREFIX_LEN), (g_au8ADCDataBuff + (u8FrameCnt * 60)), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN));
                            HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                            while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                            g_u8HIDInDone = 0;
                            memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                            g_i16HIDDataLen -= 60;
                            u8FrameCnt++;

                        } while (g_i16HIDDataLen > 0);
                    }
                }
            }
            else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
            {
                g_i16HIDDataLen = 0;
                g_u8Read_process_stop = 0;
                g_u8read_process = 0;
                g_u8_7802TimeOut = 0;
                g_u16SampleNum_cnt = 0;

                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;
                g_sHidCmdD2H.u8FrameNum = 0x00;
                g_sHidCmdD2H.u8DataLen = 0x01;
                g_sHidCmdD2H.u8CommandType = HIDTRANS_CMD_7802_CONVERTER;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
                {
                    ptr[HID_PREFIX_LEN + i] = 0;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
        }
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_Monitor_data) // for new version 7802
        {
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                uint8_t u8FrameCnt = 0;
                uint16_t i, j;
                // uint8_t ret;
                g_u16SampleNum = g_u8Sample_Num_L;

                for (i = 0; i < g_u16SampleNum; i++)
                {
                    switch (g_u8Channel_mode)
                    {
                        case HIDTRANS_CMD_CODEC_DATA_9BIT:
                            g_u16I2CTxDataLen = 1; // Write 1 byte register
                            g_u16I2CRxDataLen = 2; // Read data length is 2
                            g_au8ToI2CDataBuf[0] = g_u8RegisterAddrL << 1;
                            break;

                        case HIDTRANS_CMD_CODEC_DATA_8BIT:
                            g_u16I2CTxDataLen = 1; // Write 1 byte register
                            g_u16I2CRxDataLen = 1; // Read data length is 1
                            g_au8ToI2CDataBuf[0] = g_u8RegisterAddrL;
                            break;

                        case HIDTRANS_CMD_CODEC_DATA_16BIT:
                            g_u16I2CTxDataLen = 2; // Write 2 byte register
                            g_u16I2CRxDataLen = 2; // Read data length is 2
                            g_au8ToI2CDataBuf[0] = g_u8RegisterAddrH;
                            g_au8ToI2CDataBuf[1] = g_u8RegisterAddrL;
                            break;
                    }

                    pI2CProtocolCallback = Codec_I2CCallback;
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

                    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

                    if (g_u8T1_delay_L > 0)
                    {
                        for (j = 0; j < 1000; j++) // us * 1000 =1 ms
                        {
                            DelayUs(g_u8T1_delay_L);
                        }
                    }
                    else
                    {
                        DelayUs(500);
                    }

                    if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_DATA_8BIT)
                    {
                        g_au8Monitor_DataBuff[g_u16SampleNum_cnt * 2 + 0] = 0;
                        g_au8Monitor_DataBuff[g_u16SampleNum_cnt * 2 + 1] = g_au8ToHIDDataBuf[1];
                    }
                    else
                    {
                        g_au8Monitor_DataBuff[g_u16SampleNum_cnt * 2 + 0] = g_au8ToHIDDataBuf[0];
                        g_au8Monitor_DataBuff[g_u16SampleNum_cnt * 2 + 1] = g_au8ToHIDDataBuf[1];
                    }

                    if ((g_u16SampleNum_cnt == (g_u16SampleNum - 1)) || (g_u8WriteFail == 1))
                    {
                        g_u16SampleNum_cnt = 0;
                    }
                    else
                    {
                        g_u16SampleNum_cnt++;
                    }

                    if (g_u16SampleNum_cnt == 0)
                    {
                        g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                        g_sHidCmdD2H.u8CommandType = HIDTRANS_CMD_Monitor_data;
                        g_i16HIDDataLen = 4 + (g_u16SampleNum * 2);

                        do
                        {
                            if (g_u8WriteFail == 1)
                            {
                                g_sHidCmdD2H.u8DataLen = 3;
                                g_sHidCmdD2H.u8FrameNum = 0;
                            }
                            else
                            {
                                g_sHidCmdD2H.u8DataLen = (g_i16HIDDataLen >= 60) ? 60 : (g_i16HIDDataLen % 60);
                                g_sHidCmdD2H.u8FrameNum = u8FrameCnt;
                            }

                            if (g_i16HIDDataLen > 60)
                            {
                                g_sHidCmdD2H.u8FrameNum |= 0x80;
                            }

                            ptr = (uint8_t *)g_au8USBDmaDataBuf;
                            HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                            if (u8FrameCnt == 0)
                            {
                                if (g_u8WriteFail == 1)
                                {
                                    ptr[HID_PREFIX_LEN + 0] = !g_u8WriteFail; // 1:write success, 0: fail
                                    ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                                    ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;

                                    g_u8WriteFail = 0;
                                }
                                else
                                {
                                    ptr[HID_PREFIX_LEN + 0] = !g_u8WriteFail;
                                    ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                                    ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;
                                    ptr[HID_PREFIX_LEN + 3] = g_u16SampleNum & 0xff;
                                    HSUSBD_MemCopy((ptr + HID_PREFIX_LEN + 4), (g_au8Monitor_DataBuff), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN - 4));
                                }
                            }
                            else
                            {
                                HSUSBD_MemCopy((ptr + HID_PREFIX_LEN), (g_au8Monitor_DataBuff + (u8FrameCnt * 60) - 4), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN));
                            }

                            HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                            while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                            g_u8HIDInDone = 0;
                            memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                            g_i16HIDDataLen -= 60;
                            u8FrameCnt++;

                        } while (g_i16HIDDataLen > 0);
                    }
                }
            }
        } // end if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_Monitor_data)
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_I2C_CLK_SET)
        {
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                g_sHidCmdD2H.u8DataLen = 0x01;
                g_sHidCmdD2H.u8FrameNum = 0x00;
                g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                switch (g_u32I2C_ClockRate)
                {
                    case 100000:
                        ptr[HID_PREFIX_LEN + 0] = 0;
                        break;

                    case 400000:
                        ptr[HID_PREFIX_LEN + 0] = 1;
                        break;

                    case 1000000:
                        ptr[HID_PREFIX_LEN + 0] = 2;
                        break;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
            else // HIDTRANS_WRITE
            {
                // Save data to data flash first and then send Ack to PC
                if (g_u8ApplyFlag)
                {
                    Flash_Memory();
                    g_u8ApplyFlag = 0;
                }

                I2C_Close(I2C_PORT); // If there is no codec and init timeout, some wrong operation might occur

                pI2CProtocolCallback = NULL;

                I2C_Init();

                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;
                g_sHidCmdD2H.u8DataLen = !g_u8WriteFail;
                g_sHidCmdD2H.u8FrameNum = 0x00;
                g_sHidCmdD2H.u8CommandType = 0x00;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
                {
                    ptr[HID_PREFIX_LEN + i] = 0;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
        } // end if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_I2C_CLK_SET)
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_9BIT ||
                 g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_8BIT ||
                 g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_16BIT) // Start Codec protocol Burst mode
        {
            uint8_t u8FrameCnt = 0;

            // respond result to PC
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
                g_i16HIDDataLen = 5 + g_u16Data_Len;

                g_sHidCmdD2H.u8FrameNum = 0;
                g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                do
                {
                    g_sHidCmdD2H.u8DataLen = (g_i16HIDDataLen >= 60) ? 60 : (g_i16HIDDataLen % 60);
                    g_sHidCmdD2H.u8FrameNum = u8FrameCnt;

                    if (g_i16HIDDataLen > 60)
                    {
                        g_sHidCmdD2H.u8FrameNum |= 0x80;
                    }

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

                    if (u8FrameCnt == 0)
                    {
                        ptr[HID_PREFIX_LEN + 0] = g_u8DeviceID;
                        ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                        ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;
                        ptr[HID_PREFIX_LEN + 3] = (g_u16Data_Len >> 8) & 0xff;
                        ptr[HID_PREFIX_LEN + 4] = (g_u16Data_Len) & 0xff;
                        HSUSBD_MemCopy((ptr + HID_PREFIX_LEN + 5), (g_au8ToHIDDataBuf), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN - 5));
                    }
                    else
                    {
                        HSUSBD_MemCopy((ptr + HID_PREFIX_LEN), (g_au8ToHIDDataBuf + (u8FrameCnt * 60) - 5), (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN));
                    }

                    HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                    while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                    g_u8HIDInDone = 0;
                    memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

                    g_i16HIDDataLen -= 60;
                    u8FrameCnt++;

                } while (g_i16HIDDataLen > 0);
            }
            else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;
                g_sHidCmdD2H.u8FrameNum = 0;
                g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                if (g_u8WriteFail)
                {
                    g_sHidCmdD2H.u8DataLen = 6;

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                    ptr[HID_PREFIX_LEN + 0] = g_u8DeviceID;
                    ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                    ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;
                    ptr[HID_PREFIX_LEN + 3] = 2; // 2:Burst Fail 1:write success, 0: fail
                    ptr[HID_PREFIX_LEN + 4] = (g_u16I2CTxData_Fail_Cnt >> 8) & 0XFF;
                    ptr[HID_PREFIX_LEN + 5] = g_u16I2CTxData_Fail_Cnt;
                }
                else
                {
                    g_sHidCmdD2H.u8DataLen = 4;

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                    ptr[HID_PREFIX_LEN + 0] = g_u8DeviceID;
                    ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                    ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;
                    ptr[HID_PREFIX_LEN + 3] = !g_u8WriteFail; // 1:write success, 0: fail
                }

                g_u8WriteFail = 0;
                g_u16I2CTxData_Fail_Cnt = 0;
                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }

            // Reset buffer
            for (i = 0; i < I2C_BUFF_SIZE; i++)
            {
                g_au8ToI2CDataBuf[i] = 0;
                g_au8ToHIDDataBuf[i] = 0;
            }

            g_u16I2CRxDataLen = 0;
            g_u8HIDCommandProcessing = NOT_PROCESSING;
        } // end Codec protocol  Burst mode
        else // Start Codec protocol  Normal mode
        {
            // respond result to PC
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;

                if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BOARD_STATE) || (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BOARD_NUM))
                    g_sHidCmdD2H.u8DataLen = 1;
                else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_FW_VERSION)
                    g_sHidCmdD2H.u8DataLen = 3;
                else
                    g_sHidCmdD2H.u8DataLen = 4 + g_u16Data_Len;

                g_sHidCmdD2H.u8FrameNum = 0;
                g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                ptr = (uint8_t *)g_au8USBDmaDataBuf;
                HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                ptr[HID_PREFIX_LEN + 0] = g_u8DeviceID;
                ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;
                ptr[HID_PREFIX_LEN + 3] = g_u16Data_Len; // 2;  // Data length is 2

                switch (g_sHidCmdD2H.u8CommandType)
                {
                    case HIDTRANS_CMD_CODEC_DATA_9BIT:
                        for (i = 0; i < g_u16Data_Len; i++)
                        {
                            ptr[HID_PREFIX_LEN + 4 + i] = g_au8ToHIDDataBuf[i];
                        }

                        break;

                    case HIDTRANS_CMD_CODEC_DATA_8BIT:
                        if (g_u16Data_Len > 1)
                        {
                            for (i = 0; i < g_u16Data_Len; i++)
                            {
                                ptr[HID_PREFIX_LEN + 4 + i] = g_au8ToHIDDataBuf[i];
                            }
                        }
                        else
                        {
                            ptr[HID_PREFIX_LEN + 4] = 0;
                            ptr[HID_PREFIX_LEN + 5] = g_au8ToHIDDataBuf[0];
                        }

                        break;

                    case HIDTRANS_CMD_CODEC_DATA_16BIT:
                        for (i = 0; i < g_u16Data_Len; i++)
                        {
                            ptr[HID_PREFIX_LEN + 4 + i] = g_au8ToHIDDataBuf[i];
                        }

                        break;

                    case HIDTRANS_CMD_FW_VERSION:
                        ptr[HID_PREFIX_LEN + 0] = (g_u32PID >> 8) & 0xff;
                        ptr[HID_PREFIX_LEN + 1] = (g_u32PID >> 16) & 0xff;
                        ptr[HID_PREFIX_LEN + 2] = (g_u32PID >> 24) & 0xff;
                        break;

                    case HIDTRANS_CMD_BOARD_STATE:
                        ptr[HID_PREFIX_LEN + 0] = g8_Board_mode;
                        break;

                    case HIDTRANS_CMD_BOARD_NUM:
                        ptr[HID_PREFIX_LEN + 0] = sUserConfig.u8Default_BoardNum;
                        break;
                }

                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
            else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
            {
                g_sHidCmdD2H.u8Header = HIDTRANS_ACK;

                if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BOARD_NUM)
                {
                    g_sHidCmdD2H.u8DataLen = 1;
                    g_sHidCmdD2H.u8FrameNum = 0;
                    g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                    ptr[HID_PREFIX_LEN + 0] = !g_u8WriteFail; // 1:write success, 0: fail
                }
                else
                {
                    g_sHidCmdD2H.u8DataLen = 4;
                    g_sHidCmdD2H.u8FrameNum = 0;
                    g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

                    ptr = (uint8_t *)g_au8USBDmaDataBuf;
                    HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);
                    ptr[HID_PREFIX_LEN + 0] = g_u8DeviceID;
                    ptr[HID_PREFIX_LEN + 1] = g_u8RegisterAddrH;
                    ptr[HID_PREFIX_LEN + 2] = g_u8RegisterAddrL;
                    ptr[HID_PREFIX_LEN + 3] = !g_u8WriteFail; // 1:write success, 0: fail
                }

                g_u8WriteFail = 0;
                HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

                while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

                g_u8HIDInDone = 0;
                memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);
            }
        }

        // Reset buffer
        for (i = 0; i < I2C_BUFF_SIZE; i++)
        {
            g_au8ToI2CDataBuf[i] = 0;
            g_au8ToHIDDataBuf[i] = 0;
        }

        g_u16I2CRxDataLen = 0;
        g_u8HIDCommandProcessing = NOT_PROCESSING;
    } // End Codec protocol  Normal mode
    else if (g_u8HIDCommandProcessing == NO_DATA)
    {
        uint16_t i;
        uint8_t *ptr = (uint8_t *)g_au8USBDmaDataBuf;

        g_sHidCmdD2H.u8Header = HIDTRANS_RESPOND;
        g_sHidCmdD2H.u8DataLen = 0x00;
        g_sHidCmdD2H.u8FrameNum = 0x00;
        g_sHidCmdD2H.u8CommandType = g_sHidCmdH2D.u8CommandType;

        HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

        for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
        {
            ptr[HID_PREFIX_LEN + i] = 0;
        }

        HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

        while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

        g_u8HIDInDone = 0;
        memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

        g_u8HIDCommandProcessing = NOT_PROCESSING;
    }

    if (g_u8CheckConnection == 2)
    {
        uint8_t i;
        uint8_t *ptr = (uint8_t *)g_au8USBDmaDataBuf;

        g_sHidCmdD2H.u8Header = HIDTRANS_STATUS;
        g_sHidCmdD2H.u8DataLen = g_u8I2CTimeOut;
        g_sHidCmdD2H.u8FrameNum = 0x00;
        g_sHidCmdD2H.u8CommandType = 0x00;

        HSUSBD_MemCopy(ptr, (uint8_t *)&g_sHidCmdD2H, HID_PREFIX_LEN);

        for (i = 0; i < (EPG_MAX_PKT_SIZE - HID_PREFIX_LEN); i++)
        {
            ptr[HID_PREFIX_LEN + i] = 0;
        }

        HSUSBD_EnableEPTrans(EPG, ptr, EPG_MAX_PKT_SIZE);

        while (!g_u8HIDInDone) {} // Wait EPG data transfer to PC

        g_u8HIDInDone = 0;
        memset((void *)g_au8USBDmaDataBuf, 0, EPG_MAX_PKT_SIZE);

        if (g_u8I2CTimeOut)
        {
            g_u8HIDCommandProcessing = NOT_PROCESSING;
        }

        g_u8I2CTimeOut = 0;
        g_u8CheckConnection = 0;
    }

    AudioDeviceCtrl();

    // Check if autotrim complete
    //HIRCTrimsts = SYS->HIRCTISTS;
    HIRCTrimsts = SYS->TISTS48M;

    //if (HIRCTrimsts & SYS_HIRCTISTS_FREQLOCK_Msk)
    if (HIRCTrimsts & SYS_TISTS48M_FREQLOCK_Msk)
    {
        // turn autotrim off
        SYS->TCTL48M &= ~0x03;

        // clear flag
        SYS->TISTS48M = HIRCTrimsts;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle PDMA interrupt event                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void)
{
    int i;
    uint32_t u32Status          = PDMA_GET_INT_STATUS(PDMA_PORT);
    uint32_t u32Transferdone    = PDMA_GET_TD_STS(PDMA_PORT);
    uint32_t *puPingpongBuff    = (uint32_t *)&g_au32I2STxPingPongBuff[0];
    uint32_t *puRingBuff        = (uint32_t *)&g_au32USBPlayRingBuff[0];
    uint32_t *puUACPingpongBuff = (uint32_t *)&g_au32I2SRxPingPongBuff[0];
    uint32_t *puUACRingBuff     = (uint32_t *)&g_au32USBRecRingBuff[0];

    // write 1 to clear
    PDMA_CLR_TD_FLAG(PDMA_PORT, u32Transferdone);

    // PDMA Read/Write Target Abort Interrupt Flag
    if (u32Status & PDMA_INTSTS_ABTIF_Msk)
    {
        // PDMA Channel 2 Read/Write Target Abort Interrupt Status Flag
        if (PDMA_GET_ABORT_STS(PDMA_PORT) & BIT2)
        {
            PDMA_CLR_ABORT_FLAG(PDMA_PORT, PDMA_ABTSTS_ABTIF2_Msk);
        }
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk) // Transfer Done Interrupt Flag
    {
        // I2S Rx done
        if (u32Transferdone & BIT2)
        {
            PDMA_CLR_TD_FLAG(PDMA_PORT, PDMA_TDSTS_TDIF2_Msk);

            if (g_u8USBRecEn)
            {
                if ((I2S_PORT->CTL0 & I2S_CTL0_TDMCHNUM_Msk) == I2S_TDMCHNUM_4CH)
                {
                    if (g_u8AudioRecorderState == UAC_START_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                        g_u8AudioRecorderState = UAC_PROCESSING_AUDIO_RECORD;
                    }
                    else if (g_u8AudioRecorderState == UAC_PROCESSING_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                        g_u8AudioRecorderState = UAC_READY_AUDIO_RECORD;
                    }
                    else if (g_u8AudioRecorderState == UAC_READY_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                        g_u8AudioRecorderState = UAC_BUSY_AUDIO_RECORD;
                    }
                    else if (g_u8AudioRecorderState == UAC_BUSY_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                    }
                }
                else // I2S only 2 channel
                {
                    if (g_u8AudioRecorderState == UAC_START_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                        g_u8AudioRecorderState = UAC_PROCESSING_AUDIO_RECORD;
                    }
                    else if (g_u8AudioRecorderState == UAC_PROCESSING_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                        g_u8AudioRecorderState = UAC_READY_AUDIO_RECORD;
                    }
                    else if (g_u8AudioRecorderState == UAC_READY_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                        g_u8AudioRecorderState = UAC_BUSY_AUDIO_RECORD;
                    }
                    else if (g_u8AudioRecorderState == UAC_BUSY_AUDIO_RECORD)
                    {
                        for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
                        {
                            puUACRingBuff[g_u16Record_Write_Ptr] = puUACPingpongBuff[g_u16UAC_I2S_BUFF_LEN * g_u8I2SRxBuffIndex + i];
                            g_u16Record_Write_Ptr++;

                            if (g_u16Record_Write_Ptr == g_u16UAC_MAX_USB_BUFFER_LEN)
                            {
                                g_u16Record_Write_Ptr = 0;
                            }
                        }

                        // switch to another buffer row.
                        g_u8I2SRxBuffIndex ^= 0x1;
                    }
                }
            }
        }

        // I2S Tx done
        if (u32Transferdone & BIT1)
        {
            PDMA_CLR_TD_FLAG(PDMA_PORT, PDMA_TDSTS_TDIF1_Msk);

            if (g_u8USBPlayEn)
            {
                // When song played to end or pause, there will be no EPD anymore so Ringbuffer underflow
                // If keep last data, there will be a fixed DC on speaker, long time DC possibly makes speaker hot
                if ((g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr) && (g_bOverFlow == FALSE))
                {
                    for (i = 0; i < g_u16PlayBack_I2S_BUFF_LEN; i++)
                    {
                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = 0x0;
                    }
                }
                else
                {
                    g_bOverFlow = FALSE;

                    if (g_usbd_PlayMute)
                    {
                        for (i = 0; i < g_u16PlayBack_I2S_BUFF_LEN; i++)
                        {
                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = 0x0;
                            g_u16PlayBack_Read_Ptr++;
                        }

                        if (g_u16PlayBack_Read_Ptr >= g_u16PlayBack_MAX_USB_BUFFER_LEN)
                        {
                            g_u16PlayBack_Read_Ptr = 0;
                        }
                    }
                    else
                    {
                        if ((I2S_PORT->CTL0 & I2S_CTL0_TDMCHNUM_Msk) == I2S_TDMCHNUM_4CH)
                        {
                            for (i = 0; i < g_u16PlayBack_I2S_BUFF_LEN; i++)
                            {
                                // I2S TDM 4 channel condition
                                if ((i % 4 == 0) || (i % 4 == 1)) // channel 1 and channel 2 has data
                                {
                                    if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_16)
                                    {
                                        if (g_u8USBTxDataLen == USB_16bit)
                                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr];
                                        else if (g_u8USBTxDataLen == USB_24bit)
                                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] >> 8;

                                        g_u16PlayBack_Read_Ptr++;
                                    }
                                    else if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_24)
                                    {
                                        if (g_u8USBTxDataLen == USB_16bit)
                                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] << 8;
                                        else if (g_u8USBTxDataLen == USB_24bit)
                                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = (puRingBuff[g_u16PlayBack_Read_Ptr]);

                                        g_u16PlayBack_Read_Ptr++;
                                    }
                                    else //((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_32)
                                    {
                                        if (g_u8USBTxDataLen == USB_16bit)
                                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] << 16;
                                        else if (g_u8USBTxDataLen == USB_24bit)
                                            puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] << 8;

                                        g_u16PlayBack_Read_Ptr++;
                                    }
                                }
                                else // channel 2 and channel 4 has no data
                                {
                                    puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = 0;
                                }
                            }
                        }
                        else // I2S only 2 channel
                        {
                            for (i = 0; i < g_u16PlayBack_I2S_BUFF_LEN; i++) // 48 * 2ch
                            {
                                if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_16)
                                {
                                    if (g_u8USBTxDataLen == USB_16bit)
                                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr];
                                    else if (g_u8USBTxDataLen == USB_24bit)
                                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] >> 8;

                                    g_u16PlayBack_Read_Ptr++;
                                }
                                else if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_24)
                                {
#if (I2S_FIFO_24BIT_LSB)

                                    if (g_u8USBTxDataLen == USB_16bit)
                                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = (puRingBuff[g_u16PlayBack_Read_Ptr] << 8);
                                    else if (g_u8USBTxDataLen == USB_24bit)
                                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = (puRingBuff[g_u16PlayBack_Read_Ptr]);

#elif (I2S_FIFO_24BIT_MSB)
                                    puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] << 16;
#endif
                                    g_u16PlayBack_Read_Ptr++;
                                }
                                else //((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_32)
                                {
                                    if (g_u8USBTxDataLen == USB_16bit)
                                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] << 16;
                                    else if (g_u8USBTxDataLen == USB_24bit)
                                        puPingpongBuff[g_u16PlayBack_I2S_BUFF_LEN * g_u8I2STxBuffIndex + i] = puRingBuff[g_u16PlayBack_Read_Ptr] << 8;

                                    g_u16PlayBack_Read_Ptr++;
                                }
                            }
                        }

                        if (g_u16PlayBack_Read_Ptr >= g_u16PlayBack_MAX_USB_BUFFER_LEN)
                        {
                            g_u16PlayBack_Read_Ptr = 0;
                        }
                    }
                }

                if (g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr)
                    g_bUnderFlow = TRUE;

                // switch to another buffer row.
                g_u8I2STxBuffIndex ^= 0x1;
            }
        }
    }
}

/**
 * @brief       Flash Initial
 * @param[in]   None
 * @return      None
 * @details     Initialize the data flash and read user configuration from flash.
 */
void Flash_Init(void)
{
    uint16_t i, u16UserConfigSize;
    uint32_t *u32ptr = (uint32_t *)&sUserConfig;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel) SYS_UnlockReg();

    FMC_Open();

    u16UserConfigSize = sizeof(sUserConfig);

    for (i = 0; i < (u16UserConfigSize >> 2); i++)
    {
        *u32ptr = FMC_Read(DATA_FLASH_USERCONFIG_BASE + (i * 4)); /* Read a flash word from address DATA_FLASH_USERCONFIG_BASE+(i*4). */
        u32ptr++;
    }

    if (sUserConfig.u8Flag & 0x01) // flag == 1 means not write in yet
    {
        SetConfigToDefault();
    }

    FMC_Close();

    if (u32RegLockLevel) SYS_LockReg();
}

/**
 * @brief       Flash_Memory
 * @param[in]   None
 * @return      None
 * @details     Write user configuration to data flash.
 */
void Flash_Memory(void)
{
    uint16_t i, u16UserConfigSize;
    uint32_t *u32ptr = (uint32_t *)&sUserConfig;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
        SYS_UnlockReg();

    u16UserConfigSize = sizeof(sUserConfig);

    sUserConfig.u8Flag &= (~USER_CONFIG_MASK);

    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    FMC_Erase(DATA_FLASH_USERCONFIG_BASE); // Erase 1k data flash

    for (i = 0; i < (u16UserConfigSize >> 2); i += 2, u32ptr += 2)
    {
        FMC_Write8Bytes(DATA_FLASH_USERCONFIG_BASE + (i * 4), *(u32ptr), *(u32ptr + 1));
    }

    FMC_DISABLE_AP_UPDATE();
    FMC_Close();

    if (u32RegLockLevel)
        SYS_LockReg();
}

/**
 * @brief       PDMA Initial
 * @param[in]   None
 * @return      None
 * @details     Reset PDMA module and enable PDMA IRQ.
 */
void PDMA_Init()
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Enable PDMA irq */
    NVIC_EnableIRQ(PDMA_IRQn);
}

/**
 * @brief       HSUSBD Initial
 * @param[in]   None
 * @return      None
 * @details     Initialize the USB device, set USB descriptors and UAC interface, configure endpoints.
 */
void HSUSBD_Init(void)
{
    // USB descriptor
    extern uint8_t gu8DeviceDescriptor[];
    extern uint8_t *gpu8UsbString[];
    extern uint8_t gu8ConfigDescriptor[];
    extern uint8_t gu8ConfigDescriptor_Normal_mode_Rec_4ch[];
    extern uint32_t gu32ConfigHidDescIdx[];
    extern uint8_t gu8VendorStringDesc[];
    extern uint8_t gu8ProductStringDesc[];
    extern uint8_t gu8StringSerial[];
    uint16_t u16i;

    // Fill in Manufacture
    for (u16i = 0; u16i < 16; u16i++)
    {
        gpu8UsbString[1][u16i] = gu8VendorStringDesc[u16i];
    }

    // Fill in Product
    for (u16i = 0; u16i < 46; u16i++)
    {
        gpu8UsbString[2][u16i] = gu8ProductStringDesc[u16i];
    }

    // Fill in SerialNumber
    for (u16i = 0; u16i < 26; u16i++)
    {
        gpu8UsbString[3][u16i] = gu8StringSerial[u16i];
    }

    if (g8_Board_mode == Board_Normal)
    {
        if (g_u32I2S0Channel == I2S_TDMCHNUM_2CH)
        {
            gsHSInfo.gu8ConfigDesc = gu8ConfigDescriptor;
        }
        else
        {
            gsHSInfo.gu8ConfigDesc = gu8ConfigDescriptor_Normal_mode_Rec_4ch;
            gu8DeviceDescriptor[8] = USBD_VID & 0xFF;
            gu8DeviceDescriptor[9] = (USBD_VID >> 8) & 0xFF;
            gu8DeviceDescriptor[10] = (USBD_PID + 2) & 0xFF; // USBD_PID =0x120D
            gu8DeviceDescriptor[11] = (USBD_PID >> 8) & 0xFF;
        }
    }

    // HSUSBD initial
    HSUSBD_Open(&gsHSInfo, UAC_ClassRequest, (HSUSBD_SET_INTERFACE_REQ)UAC_SetInterface);

    // Endpoint configuration
    UAC_Init();

    NVIC_EnableIRQ(HSUSBD_IRQn);
}

/**
 * @brief       I2C Initial
 * @param[in]   None
 * @return      None
 * @details     Reset I2C module, set I2C bus clock and enable I2C interrupt.
 */
void I2C_Init(void)
{
    if (g8_Board_mode == Board_Normal)
    {
        g_u32I2C_ClockRate = 400000;
    }

    switch (sUserConfig.u8I2C_Address)
    {
        case I2C_Addr_0x40:
            g_u8SlaveAddr = 0x40;
            break;

        case I2C_addr_0x42:
            g_u8SlaveAddr = 0x42;
            break;

        case I2C_Addr_0x44:
            g_u8SlaveAddr = 0x44;
            break;

        case I2C_Addr_0x46:
            g_u8SlaveAddr = 0x46;
            break;
    }

    /* Reset module */
    SYS_ResetModule(SYS_I2C3RST);

    /* Open I2C module and set bus clock */
    I2C_Open(I2C_PORT, g_u32I2C_ClockRate);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C_PORT);

    NVIC_ClearPendingIRQ(I2C3_IRQn);

    NVIC_EnableIRQ(I2C3_IRQn);
}

/**
 * @brief       I2S Initial
 * @param[in]   None
 * @return      None
 * @details     Initialize the I2S interface and set the I2S related parameters.
 */
//#define I2S_MASTER_SAMPLE_RATE  (USB_RATE)
//#define I2S_CLOCK_SOURCE_HIRC_48M
void I2S_Init(void)
{
    // flag == 1 means not write in yet
    if ((sUserConfig.u8Flag & 0x01) || (g8_Board_mode == Board_Normal))
    {
        /* Default setting */
        g_u32I2S0SampleRate = 48000;
        g_u32I2S0SampleRate_old = g_u32I2S0SampleRate;
        g_u32I2S0Channel = I2S_TDMCHNUM_2CH;
        g_u32I2S0Mono = I2S_STEREO;
        g_u32I2S0SerialMode = I2S_MODE_SLAVE;
        g_u32I2S0Format = I2S_FORMAT_I2S;
        g_u32I2S0WordSize = I2S_DATABIT_24;
        g_u32I2S0FrameSize = I2S_CHWIDTH_32;
    }

    PlaybackBuffInit();

    RecordBuffInit();

    if ((g_u32I2S0Channel == I2S_TDMCHNUM_4CH) && (g_u32I2S0SampleRate == 32000))
        g_u8UAC_ADAPTIVE = 1;

    CLK_EnableModuleClock(I2S_MODULE);

    // Select I2S clock.
    CLK_SetModuleClock(I2S_MODULE, I2S_CLKSRC, 0);

    // I2S IPReset.
    SYS_ResetModule(I2S_RST);

    // Open I2S hardware IP
    I2S_open(I2S_PORT, g_u32I2S0SerialMode, g_u32I2S0SampleRate, g_u32I2S0WordSize, g_u32I2S0Channel, g_u32I2S0Mono, g_u32I2S0Format, g_u32I2S0FrameSize);


    if (g_u32I2S0WordSize == I2S_DATABIT_16)
    {
        I2S_SET_PBWIDTH(I2S_PORT, I2S_PBWIDTH_16);
        I2S_SET_PB16ORD(I2S_PORT, I2S_PB16ORD_LOW);
    }
    else
    {
        I2S_SET_PBWIDTH(I2S_PORT, I2S_PBWIDTH_32);
    }

    // I2S Configuration
    I2S_SET_MONO_RX_CHANNEL(I2S_PORT, I2S_MONO_RIGHT);

    if (g_u32I2S0WordSize == I2S_DATABIT_16)
    {
        // Right channel at RXFIFO[31:24], Left channel at RXFIFO[15:0]
        I2S_SET_STEREOORDER(I2S_PORT, I2S_ORDER_EVENLOW);
    }
    else
    {
#if (I2S_FIFO_24BIT_LSB)
        // LSB of 24-bit audio data in each channel is aligned to right side in 32-bit FIFO entries.
        I2S_SET_STEREOORDER(I2S_PORT, I2S_ORDER_EVENHIGH);
#elif (I2S_FIFO_24BIT_MSB)
        // MSB of 24-bit audio data in each channel is aligned to left side in 32-bit FIFO entries.
        I2S_SET_STEREOORDER(I2S_PORT, I2S_ORDER_EVENLOW);
#endif
    }

    // Set channel width.
    I2S_SET_CHWIDTH(I2S_PORT, g_u32I2S0FrameSize);

    if (g_u32I2S0SerialMode == I2S_MODE_MASTER)
    {
        I2S_EnableMCLK(I2S_PORT, g_u32I2S0SampleRate * 500); //MCLK = 24M, with CLK_CLKSEL3_I2S0SEL_HIRC
    }
    else
    {
        I2S_EnableMCLK(I2S_PORT, 12000000); // MCLK = 12M, slave mode
    }
}

/**
 * @brief       Systick Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Initialize the system tick and enable SysTick interrupt.
 */
void Systick_Init(void)
{
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000)) // 1ms
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1) {}
    }

    NVIC_SetPriority(SysTick_IRQn, 1);
}

/**
 * @brief       PDMA_I2S_RX_Init
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Initialize the PDMA channel for I2S0 RX request.
 */
void PDMA_I2S_RX_Init(void)
{
    DMA_RXDESC[0].ctl = (((g_u16UAC_I2S_BUFF_LEN) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | // transfer byte count
                        PDMA_WIDTH_32 |
                        PDMA_SAR_FIX |    // Source Address Increment
                        PDMA_DAR_INC |    // Destination Address fixed
                        PDMA_REQ_SINGLE | // Transfer Type = single
                        PDMA_OP_SCATTER;  // Basic mode
    DMA_RXDESC[0].endsrc = (uint32_t)&I2S_PORT->RXFIFO;
    DMA_RXDESC[0].enddest = (uint32_t)&g_au32I2SRxPingPongBuff[0];
    DMA_RXDESC[0].offset = ((uint32_t)&DMA_RXDESC[1]);

    DMA_RXDESC[1].ctl = (((g_u16UAC_I2S_BUFF_LEN) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | // transfer byte count
                        PDMA_WIDTH_32 |
                        PDMA_SAR_FIX |    // Source Address Increment
                        PDMA_DAR_INC |    // Destination Address fixed
                        PDMA_REQ_SINGLE | // Transfer Type = single
                        PDMA_OP_SCATTER;  // Basic mode
    DMA_RXDESC[1].endsrc = (uint32_t)&I2S_PORT->RXFIFO;
    DMA_RXDESC[1].enddest = (uint32_t)&g_au32I2SRxPingPongBuff[g_u16UAC_I2S_BUFF_LEN];
    DMA_RXDESC[1].offset = ((uint32_t)&DMA_RXDESC[0]);

    // Open PDMA channel
    PDMA_Open(PDMA_PORT, PDMA_CH2_MASK);
    // Request source is memory to memory
    PDMA_SetTransferMode(PDMA_PORT, 2, PDMA_I2S0_RX, TRUE, DMA_RXDESC[1].offset);
    // Enable PDMA channel 1 interrupt
    PDMA_EnableInt(PDMA_PORT, 2, PDMA_INT_TRANS_DONE);
}

/**
 * @brief       PDMA_I2S_Tx_Init
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Initialize the PDMA channel for I2S0 TX request.
 */
void PDMA_I2S_Tx_Init(void)
{
    DMA_TXDESC[0].ctl = (((g_u16PlayBack_I2S_BUFF_LEN) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | // transfer byte count
                        PDMA_WIDTH_32 |
                        PDMA_SAR_INC |    // Source Address Increment
                        PDMA_DAR_FIX |    // Destination Address fixed
                        PDMA_REQ_SINGLE | // Transfer Type = single
                        PDMA_OP_SCATTER;  // Basic mode
    DMA_TXDESC[0].endsrc = (uint32_t)&g_au32I2STxPingPongBuff[0];
    DMA_TXDESC[0].enddest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_TXDESC[0].offset = ((uint32_t)&DMA_TXDESC[1]);

    DMA_TXDESC[1].ctl = (((g_u16PlayBack_I2S_BUFF_LEN) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | // transfer byte count
                        PDMA_WIDTH_32 |
                        PDMA_SAR_INC |    // Source Address Increment
                        PDMA_DAR_FIX |    // Destination Address fixed
                        PDMA_REQ_SINGLE | // Transfer Type = single
                        PDMA_OP_SCATTER;  // Basic mode
    DMA_TXDESC[1].endsrc = (uint32_t)&g_au32I2STxPingPongBuff[g_u16PlayBack_I2S_BUFF_LEN];
    DMA_TXDESC[1].enddest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_TXDESC[1].offset = ((uint32_t)&DMA_TXDESC[0]);

    // Open PDMA channel
    PDMA_Open(PDMA_PORT, PDMA_CH1_MASK);
    // Request source is memory to memory
    PDMA_SetTransferMode(PDMA_PORT, 1, PDMA_I2S0_TX, TRUE, DMA_TXDESC[1].offset);
    // Enable PDMA channel 1 interrupt
    PDMA_EnableInt(PDMA_PORT, 1, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA_IRQn);
}

/**
 * @brief       I2S_TX_Start
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Start I2S TX function and enable TX DMA function.
 */
void I2S_TX_Start(void)
{
    PDMA_I2S_Tx_Init();

    I2S_CLR_TX_FIFO(I2S_PORT);
    I2S_ENABLE_TXDMA(I2S_PORT);

    I2S_PORT->CTL0 |= I2S_CTL0_I2SEN_Msk;
}

/**
 * @brief       I2S_TX_Stop
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Stop I2S TX function and disable TX DMA function.
 */
void I2S_TX_Stop(void)
{
    PDMA_PORT->CHCTL &= ~(1ul << 1);
    PDMA_PORT->CHRST |= (1ul << 1);

    PDMA_CLR_TD_FLAG(PDMA_PORT, (1 << 1));
    I2S_DISABLE_TXDMA(I2S_PORT);
    I2S_DISABLE_TX(I2S_PORT);
    I2S_CLR_TX_FIFO(I2S_PORT);

    g_u8I2STxBuffIndex = 0;
    g_u16PlayBack_Read_Ptr = 0;
    g_u16PlayBack_Write_Ptr = 0;

    PlaybackPingpongBuffClear();
}

/**
 * @brief       I2S_RX_Start
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Start I2S RX function and enable RX DMA function.
 */
void I2S_RX_Start(void)
{
    PDMA_I2S_RX_Init();

    I2S_CLR_RX_FIFO(I2S_PORT);
    I2S_PORT->CTL0 |= I2S_CTL0_I2SEN_Msk;

    I2S_ENABLE_RX(I2S_PORT);
    I2S_ENABLE_RXDMA(I2S_PORT);
}

/**
 * @brief       I2S_RX_Stop
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Stop I2S RX function and disable RX DMA function.
 */
void I2S_RX_Stop(void)
{
    PDMA_PORT->CHCTL &= ~(1ul << 2);
    PDMA_PORT->CHRST |= (1ul << 2);
    PDMA_CLR_TD_FLAG(PDMA_PORT, (1 << 2));

    I2S_DISABLE_RXDMA(I2S_PORT);
    I2S_DISABLE_RX(I2S_PORT);
    I2S_CLR_RX_FIFO(I2S_PORT);

    g_u8I2SRxBuffIndex = 0;
    g_u16Record_Read_Ptr = 0;
    g_u16Record_Write_Ptr = 0;

    RecordPingpongBuffClear();
}

/**
 * @brief       Switch_I2S_Sample_Rate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Switch I2S sample rate and re-configure the I2S0 related parameters.
 */
void Switch_I2S_Sample_Rate(void)
{
    // stop I2S TX/RX fuction
    I2S_TX_Stop();
    I2S_RX_Stop();

    CLK_SetModuleClock(I2S_MODULE, I2S_CLKSRC, 0);

    I2S_open(I2S_PORT, g_u32I2S0SerialMode, g_u32I2S0SampleRate, g_u32I2S0WordSize, g_u32I2S0Channel, g_u32I2S0Mono, g_u32I2S0Format, g_u32I2S0FrameSize);

    SYS_ResetModule(I2S_RST);

    // Set PDMA bus width
    if (g_u32I2S0WordSize == I2S_DATABIT_16)
    {
        I2S_SET_PBWIDTH(I2S_PORT, I2S_PBWIDTH_16);
        I2S_SET_PB16ORD(I2S_PORT, I2S_PB16ORD_LOW);
        I2S_SET_STEREOORDER(I2S_PORT, I2S_ORDER_EVENLOW);
    }
    else
    {
        I2S_SET_PBWIDTH(I2S_PORT, I2S_PBWIDTH_32);
#if (I2S_FIFO_24BIT_LSB)
        // LSB of 24-bit audio data in each channel is aligned to right side in 32-bit FIFO entries.
        I2S_SET_STEREOORDER(I2S_PORT, I2S_ORDER_EVENHIGH);
#elif (I2S_FIFO_24BIT_MSB)
        // MSB of 24-bit audio data in each channel is aligned to left side in 32-bit FIFO entries.
        I2S_SET_STEREOORDER(I2S_PORT, I2S_ORDER_EVENLOW);
#endif
    }

    // Set channel width.
    I2S_SET_CHWIDTH(I2S_PORT, g_u32I2S0FrameSize);

    if (g_u32I2S0SerialMode == I2S_MODE_MASTER)
    {
        I2S_EnableMCLK(I2S_PORT, g_u32I2S0SampleRate * 500);        //MCLK = 24M, with CLK_CLKSEL3_I2S0SEL_HIRC
    }
    else
    {
        I2S_EnableMCLK(I2S_PORT, 12000000); // MCLK = 12M, slave mode
    }
}

/**
 * @brief       Enable EP IN transfer
 *
 * @param[in]   epnum   Endpoint number
 *
 * @param[in]   *buf    The pointer of data buffer
 *
 * @param[in]   len     The length of data
 *
 * @return      None
 *
 * @details     Enable EP IN transfer function and start USB IN transfer.
 */
void HSUSBD_EnableEPTrans(uint32_t epnum, uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        HSUSBD->EP[epnum].EPDAT_BYTE = buf[i];
    }

    HSUSBD->EP[epnum].EPTXCNT = len;

    HSUSBD_ENABLE_EP_INT(epnum, HSUSBD_EPINTEN_INTKIEN_Msk);
}

/**
 * @brief       I2S_open
 *
 * @param[in]   *i2s            The pointer of I2S module
 * @param[in]   u32MasterSlave  The operation mode of I2S module
 * @param[in]   u32SampleRate   The sample rate of I2S module
 * @param[in]   u32WordWidth    The word width of I2S module
 * @param[in]   u32Channels     The channel number of I2S module
 * @param[in]   u32Mono         The mono or stereo mode of I2S module
 * @param[in]   u32DataFormat   The data format of I2S module
 * @param[in]   u32I2S0FrameSize The frame size of I2S module
 *
 * @return      The real sample rate of I2S module
 *
 * @details     This function is used to configure the I2S controller and calculate the divider.
 */
static uint32_t I2S_open(I2S_T *i2s,
                         uint32_t u32MasterSlave,
                         uint32_t u32SampleRate,
                         uint32_t u32WordWidth,
                         uint32_t u32Channels,
                         uint32_t u32Mono,
                         uint32_t u32DataFormat,
                         uint32_t u32I2S0FrameSize)
{
    uint32_t u32Divider;
    uint32_t u32BitRate, u32SrcClk;

    /* Reset SPI/I2S */
    if (i2s == I2S0)
        SYS_ResetModule(SYS_I2S0RST);
    else
        SYS_ResetModule(SYS_I2S1RST);

    /* Configure I2S controller */
    i2s->CTL0 = u32MasterSlave | u32WordWidth | u32Mono | u32DataFormat;
    /* Set TDM channel number */
    I2S_SET_TDMCHNUM(i2s, u32Channels);

    /* Check Mono, Format, Channel, Datawidth setting*/
    if (u32Mono == I2S_MONO)
        u32Channels = 2;
    else
    {
        if (u32DataFormat == I2S_FORMAT_PCM || u32DataFormat == I2S_FORMAT_PCM_MSB || u32DataFormat == I2S_FORMAT_PCM_LSB)
            u32Channels = ((u32Channels >> I2S_CTL0_TDMCHNUM_Pos) + 1) * 2;
        else
            u32Channels = 2;
    }

    u32WordWidth = (u32WordWidth >> I2S_CTL0_DATWIDTH_Pos) + 1;
    u32I2S0FrameSize = (u32I2S0FrameSize >> I2S_CTL0_CHWIDTH_Pos) + 1;

    if (u32WordWidth == 1 && u32Channels > 2)
        u32WordWidth = 2;

    if (u32MasterSlave == I2S_MODE_MASTER)
    {
        uint32_t u32Error, u32Error2;

        /* Get the source clock rate */
        u32SrcClk = I2S_GetSourceClockFreq(i2s);

        u32BitRate = u32SampleRate * u32I2S0FrameSize * 8 * u32Channels;
        u32Divider = ((u32SrcClk / u32BitRate) >> 1) - 1;
        /* Adjust Error */
        u32Error = (u32SrcClk / (2 * (u32Divider + 1)) - u32BitRate);
        u32Error2 = (u32BitRate - u32SrcClk / (2 * (u32Divider + 2)));
        u32Divider = (u32Error < u32Error2) ? u32Divider : (u32Divider + 1);

        /* Set BCLKDIV setting */
        i2s->CLKDIV = (i2s->CLKDIV & ~I2S_CLKDIV_BCLKDIV_Msk) | (u32Divider << I2S_CLKDIV_BCLKDIV_Pos);

        /* Calculate bit clock rate */
        u32BitRate = u32SrcClk / ((u32Divider + 1) * 2);
        /* Calculate real sample rate */
        u32SampleRate = u32BitRate / (u32I2S0FrameSize * 8 * u32Channels);
    }
    else
    {
        /* Set BCLKDIV = 0 */
        i2s->CLKDIV &= ~I2S_CLKDIV_BCLKDIV_Msk;
        /* Get the source clock rate */
        u32SampleRate = I2S_GetSourceClockFreq(i2s);
    }

    /* Return the real sample rate */
    return u32SampleRate;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
