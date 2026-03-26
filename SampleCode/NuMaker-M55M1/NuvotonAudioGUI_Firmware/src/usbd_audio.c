/******************************************************************************
 * @file     usbd_audio.c
 * @version  V1.00
 * @brief    HSUSBD HID and audio codec sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "usbd_audio.h"
#include "user_config.h"
#include "i2c_process.h"
#include "codec_config.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
// For Control Pipe
volatile uint8_t g_u8AudioSpeakerState = 0;
volatile uint8_t g_u8AudioRecorderState = 0;

// USB flow control variables
uint32_t g_u32BulkBuf0, g_u32BulkBuf1;
static uint32_t g_u32SpeakerByteCount, g_u32SpeakerByteCountRec;
static uint8_t g_u8AudioSpeakerDebounce;

// USB Buffer Control Variable
volatile uint32_t g_au32USBPlayRingBuff[(USB_RATE / 1000) * RING_BUFF_LEVEL * PLAY_4CHANNELS * AMIC_DATASIZE_BC] __attribute__((aligned(4)));
volatile uint32_t g_au32USBRecRingBuff[(USB_RATE / 1000) * RING_BUFF_LEVEL * PLAY_4CHANNELS * AMIC_DATASIZE_BC] __attribute__((aligned(4)));
volatile uint32_t g_au32I2STxPingPongBuff[(USB_RATE / 1000) * PLAY_4CHANNELS * AMIC_DATASIZE_BC * 2] __attribute__((aligned(4)));
volatile uint32_t g_au32I2SRxPingPongBuff[(USB_RATE / 1000) * PLAY_4CHANNELS * AMIC_DATASIZE_BC * 2] __attribute__((aligned(4)));
volatile uint8_t g_au8IntOutBuff[EPH_MAX_PKT_SIZE] __attribute__((aligned(4))) = {0};
volatile uint8_t g_au8IsoOutBuff[EPD_MAX_PKT_SIZE_24bit] __attribute__((aligned(4))) = {0};
volatile uint8_t g_au8IsoInBuff[EPC_MAX_PKT_SIZE] __attribute__((aligned(4))) = {0};
volatile uint8_t g_u8I2STxBuffIndex = 0;
volatile uint8_t g_u8I2SRxBuffIndex = 0;
volatile uint16_t g_u16PlayBack_Read_Ptr = 0;
volatile uint16_t g_u16PlayBack_Write_Ptr = 0;
volatile uint16_t g_u16PlayBack_Ptrs_Distance = 0;
volatile uint16_t g_u16Record_Read_Ptr = 0;
volatile uint16_t g_u16Record_Write_Ptr = 0;
volatile uint16_t g_u16PayLoadLen;
volatile uint32_t g_u32I2C_ClockRate = 400000;

// placyabck
volatile uint16_t g_u16PlayBack_MAX_USB_BUFFER_LEN = 0;
volatile uint16_t g_u16PlayBack_I2S_BUFF_LEN = 0;
volatile uint16_t g_u16PlayBack_USB_BUFFER_THRE_BASE = 0;
volatile uint16_t g_u16PlayBack_USB_BUFF_UPPER_THRE = 0;
volatile uint16_t g_u16PlayBack_USB_BUFF_LOWER_THRE = 0;
volatile uint32_t g_u32I2S0SampleRate_old = PLAY_RATE_48K;
// UAC
volatile uint16_t g_u16UAC_MAX_USB_BUFFER_LEN = 0;
volatile uint16_t g_u16UAC_I2S_BUFF_LEN = 0;
volatile uint16_t g_u16UAC_USB_BUFFER_THRE_BASE = 0;
volatile uint16_t g_u16UAC_USB_BUFF_UPPER_THRE = 0;
volatile uint16_t g_u16UAC_USB_BUFF_LOWER_THRE = 0;

// Audio Parameter
volatile uint32_t g_usbd_PlaySampleRate = PLAY_RATE_48K;
volatile uint32_t g_usbd_RecSampleRate = REC_RATE_48K;
volatile uint8_t g_u8USBTxDataLen = USB_16bit;      // Only support 16-bit now
volatile uint8_t g_usbd_PlayMute      = 0x00;       /* Play MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_PlayVolumeL   = 0x1000;     /* Play left channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_PlayVolumeR   = 0x1000;     /* Play right channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_PlayMaxVolume = 0x7FFF;
volatile int16_t g_usbd_PlayMinVolume = 0x8000;
volatile int16_t g_usbd_PlayResVolume = 0x400;

volatile uint8_t g_usbd_RecMute       = 0x00;       /* Record MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_RecVolumeL    = 0x1000;     /* Record left channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_RecVolumeR    = 0x1000;     /* Record right channel volume. Range is -32768 ~ 32767 */
volatile int16_t g_usbd_RecMaxVolume  = 0x7FFF;
volatile int16_t g_usbd_RecMinVolume  = 0x8000;
volatile int16_t g_usbd_RecResVolume  = 0x400;

volatile uint32_t g_u32Flag = 0;

uint8_t g_au8ToHIDDataBuf[I2C_BUFF_SIZE] = {0}; // Data to PC; [0][1]:I2C data length, [2]:command type, [3] feedback or not, the rest are data
uint8_t g_au8ToI2CDataBuf[I2C_BUFF_SIZE] = {0}; // Data from PC; [0][1]:I2C data length, [2]:command type, [3] feedback or not, the rest are data
uint8_t g_au8USBDmaDataBuf[EPG_MAX_PKT_SIZE] = {0};

// HID transfer command handler
HIDTRANS_CMD_T g_sHidCmdH2D;  // Host to device
HIDTRANS_CMD_T g_sHidCmdD2H;  // Device to host

// HID flag
volatile uint8_t g_u8HIDCommandProcessing = NOT_PROCESSING;
volatile uint8_t g_u8HIDInDone = 0;
volatile uint8_t g_u8BongioviWriteReceived = 0;

// Codec Info
volatile uint8_t g_u8DeviceID;
volatile uint8_t g_u8RegisterAddrH;
volatile uint8_t g_u8RegisterAddrL;
volatile uint8_t g_u8BurstLen_H;
volatile uint8_t g_u8BurstLen_L;
volatile uint16_t g_u16Data_Len;
volatile uint8_t g_u8WriteFail;
volatile uint16_t g_u16I2CTxData_Fail_Cnt;

// NAU7802 Info
volatile uint8_t g_u8Qurey_mode;
volatile uint8_t g_u8Channel_mode;
volatile uint8_t g_u8Sample_Num_H;
volatile uint8_t g_u8Sample_Num_L;
volatile uint8_t g_u8T1_delay_H;
volatile uint8_t g_u8T1_delay_L;
volatile uint8_t g_u8T2_delay_H;
volatile uint8_t g_u8T2_delay_L;
volatile uint8_t g_u8Read_process_stop = 0;
volatile uint8_t g_u8read_process = 0;
volatile uint16_t g_u16SampleNum_cnt = 0;
uint8_t g_au8ADCDataBuff[6144] = {0};  //max 1024(sample) * 3(byte) * 2 (ch)

// I2C related parameter
extern volatile uint8_t g_u8CheckConnection;
extern volatile uint8_t g_u8I2CTimeOut;

// I2S parameter
extern volatile uint32_t g_u32I2S0SampleRate;
extern volatile uint32_t g_u32I2S0Channel;
extern volatile uint32_t g_u32I2S0Channel_old;
extern volatile uint32_t g_u32I2S0SerialMode;
extern volatile uint32_t g_u32I2S0Format;
extern volatile uint32_t g_u32I2S0Mono;
extern volatile uint32_t g_u32I2S0WordSize;
extern volatile uint32_t g_u32I2S0FrameSize;
extern volatile uint8_t g_u8USBPlayEn;
extern volatile uint8_t g_u8USBRecEn;
extern volatile uint8_t g_bUnderFlow; // No RingBuffer for reading
extern volatile uint8_t g_bOverFlow;  // No RingBuffer for Writting
extern volatile uint8_t g8_Board_mode;

volatile uint8_t g_u8UAC_ADAPTIVE = 0;

/**
 * @brief       Playback ping pong buffer clear
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to clear content of playback ping pong buffer
 */
void PlaybackPingpongBuffClear(void)
{
    uint32_t i;

    for (i = 0; i < g_u16PlayBack_I2S_BUFF_LEN; i++)
    {
        g_au32I2STxPingPongBuff[i] = 0;
        g_au32I2STxPingPongBuff[g_u16PlayBack_I2S_BUFF_LEN + i] = 0;
    }
}

/**
 * @brief       Record ping pong buffer clear
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to clear content of record ping pong buffer
 */
void RecordPingpongBuffClear(void)
{
    uint32_t i;

    for (i = 0; i < g_u16UAC_I2S_BUFF_LEN; i++)
    {
        g_au32I2SRxPingPongBuff[i] = 0;
        g_au32I2SRxPingPongBuff[g_u16UAC_I2S_BUFF_LEN + i] = 0;
    }
}

/**
 * @brief       Playback ring & Record ring buffer initiate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to set playback ring & Record ring buffer threshold and pointer
 */
void PlaybackBuffInit(void)
{
    uint8_t I2S0Channel;

    if (g_u32I2S0Channel == I2S_TDMCHNUM_2CH)
    {
        I2S0Channel = 2;
    }
    else
    {
        I2S0Channel = 4;
    }

    g_u16PlayBack_Read_Ptr = 0;
    g_u16PlayBack_Write_Ptr = 0;
    g_u16PlayBack_I2S_BUFF_LEN = (g_u32I2S0SampleRate * I2S0Channel * 2 / 1000);
    g_u16PlayBack_USB_BUFFER_THRE_BASE = (g_u32I2S0SampleRate * I2S0Channel / 1000);
    g_u16PlayBack_MAX_USB_BUFFER_LEN = g_u16PlayBack_USB_BUFFER_THRE_BASE * 10;
    g_u16PlayBack_USB_BUFF_UPPER_THRE = g_u16PlayBack_USB_BUFFER_THRE_BASE * 7;
    g_u16PlayBack_USB_BUFF_LOWER_THRE = g_u16PlayBack_USB_BUFFER_THRE_BASE * 4;
}

/**
 * @brief       Record ring buffer initiate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to set record ring buffer threshold and pointer
 */
void RecordBuffInit(void)
{
    uint8_t I2S0Channel;

    if (g_u32I2S0Channel == I2S_TDMCHNUM_2CH)
    {
        I2S0Channel = 2;
    }
    else
    {
        I2S0Channel = 4;
    }

    g_u16Record_Write_Ptr = 0;
    g_u16Record_Read_Ptr = 0;
    g_u16UAC_I2S_BUFF_LEN = (g_u32I2S0SampleRate * I2S0Channel * 2 / 1000);
    g_u16UAC_USB_BUFFER_THRE_BASE = (g_u32I2S0SampleRate * I2S0Channel / 1000);
    g_u16UAC_MAX_USB_BUFFER_LEN = g_u16UAC_USB_BUFFER_THRE_BASE * 10;
    g_u16UAC_USB_BUFF_UPPER_THRE = g_u16UAC_USB_BUFFER_THRE_BASE * 7;
    g_u16UAC_USB_BUFF_LOWER_THRE = g_u16UAC_USB_BUFFER_THRE_BASE * 4;
}

/**
 * @brief       HID Command Process
 * @param[in]   pu8Buffer    The pointer of buffer to store command from PC
 * @param[in]   u32BufferLen The length of buffer
 * @return      None
 * @details     This function is used to process HID command from PC.
 */
void HIDTrans_ProcessCommand(uint8_t *pu8Buffer, uint32_t u32BufferLen)
{
    (void)u32BufferLen;

    if ((g_u8HIDCommandProcessing == NOT_PROCESSING) || (g_u8HIDCommandProcessing == WAITING_MORE_DATA))
    {
        uint8_t u8FrameCnt;
        uint16_t i;

        // Copy the prefix for analyzation
        HSUSBD_MemCopy((uint8_t *)&g_sHidCmdH2D, pu8Buffer, HID_PREFIX_LEN);

        if (g_sHidCmdH2D.u8Header == HIDTRANS_CHECK)
        {
            // Write dummy data to check I2C connection
            g_u8CheckConnection = 1;
            pI2CProtocolCallback = I2C_Callback;
            I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA);
            return;
        }

        if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BONGIOVI) && (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE))
        {
            g_u8BongioviWriteReceived = 1;
        }

        if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BONGIOVI) && (g_sHidCmdH2D.u8Header == HIDTRANS_READ))
        {
            if (!g_u8BongioviWriteReceived)
            {
                // No write command before, therefore no data to send
                g_u8HIDCommandProcessing = NO_DATA;
                return;
            }
        }

        if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_FW_VERSION) && (g_sHidCmdH2D.u8Header == HIDTRANS_READ))
        {
            g_u8HIDCommandProcessing = FINISHED;
            return;
        }

        if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BOARD_STATE) && (g_sHidCmdH2D.u8Header == HIDTRANS_READ))
        {
            g_u8HIDCommandProcessing = FINISHED;
            return;
        }

        if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BOARD_NUM)
        {
            if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
            {
                sUserConfig.u8Default_BoardNum = pu8Buffer[4];
                Flash_Memory();
            }

            g_u8HIDCommandProcessing = FINISHED;
            return;
        }

        if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_7802_CONVERTER) // for new version 7802
        {
            GPIO_SetMode(PD, BIT4, GPIO_MODE_QUASI);

            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_u8Qurey_mode = pu8Buffer[4];   // Polling or Time-delay
                g_u8Channel_mode = pu8Buffer[5]; // single or dual
                g_u8Sample_Num_H = pu8Buffer[6]; // Sample number high byte
                g_u8Sample_Num_L = pu8Buffer[7]; // sample number low byte

                if (g_sHidCmdH2D.u8DataLen == 4)
                {
                    g_u8T1_delay_H = 0; // T1 delay High Byte
                    g_u8T1_delay_L = 0; // T1 delay Low Byte
                    g_u8T2_delay_H = 0; // T2 delay High Byte
                    g_u8T2_delay_L = 0; // T2 delay Low Byte
                }
                else if (g_sHidCmdH2D.u8DataLen == 6)
                {
                    g_u8T1_delay_H = pu8Buffer[8]; // T1 delay High Byte
                    g_u8T1_delay_L = pu8Buffer[9]; // T1 delay Low Byte
                }
                else if (g_sHidCmdH2D.u8DataLen == 8)
                {
                    g_u8T1_delay_H = pu8Buffer[8];  // T1 delay High Byte
                    g_u8T1_delay_L = pu8Buffer[9];  // T1 delay Low Byte
                    g_u8T2_delay_H = pu8Buffer[10]; // T2 delay High Byte
                    g_u8T2_delay_L = pu8Buffer[11]; // T2 delay Low Byte
                }
            }
            else if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
            {
                g_u8Qurey_mode = pu8Buffer[4]; // Interrupt

                g_u8Read_process_stop = 1;
            }

            g_u8HIDCommandProcessing = FINISHED;
            return;
        }

        // ---------- I2C protocol start ----------
        if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BONGIOVI ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DPS_SETTING ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DPS_STRING ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DATA_CONFIG ||
                g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_DATA_PRESET)
        {
            printf("I2C Set\r\n");
            // Copy data into ToI2C buffer
            u8FrameCnt = g_sHidCmdH2D.u8FrameNum & 0x7F;
            HSUSBD_MemCopy((g_au8ToI2CDataBuf + I2C_PREFIX_LEN + (u8FrameCnt * 60)), (pu8Buffer + HID_PREFIX_LEN), g_sHidCmdH2D.u8DataLen);
            g_u16I2CTxDataLen += (g_sHidCmdH2D.u8DataLen);

            if (g_sHidCmdH2D.u8FrameNum & 0x80) // The command has more than 60 data
            {
                g_u8HIDCommandProcessing = WAITING_MORE_DATA;
                return;
            }
            else
            {
                g_u8HIDCommandProcessing = PROCESSING;
            }

            if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON)
            {
                g_u16I2CTxDataLen = g_au8ToI2CDataBuf[6] + HID_HOST_PREFIX_LEN + I2C_PREFIX_LEN; // only send actual data
            }
            else
            {
                g_u16I2CTxDataLen = g_u16I2CTxDataLen + I2C_PREFIX_LEN;
            }

            g_u16I2CTxDataCnt = 0; // Count how many I2C data had been transmitted
            g_au8ToI2CDataBuf[0] = g_u16I2CTxDataLen & 0x00FF;
            g_au8ToI2CDataBuf[1] = (g_u16I2CTxDataLen >> 8) & 0x00FF;
            g_au8ToI2CDataBuf[2] = g_sHidCmdH2D.u8CommandType;

            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_au8ToI2CDataBuf[3] = 0x01;
            }
            else
            {
                g_au8ToI2CDataBuf[3] = 0x00;
            }

            // Analyze Nnuvoton command to for action
            if ((g_sHidCmdH2D.u8Header == HIDTRANS_WRITE) && (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON))
            {
                NuvotonCmdAnalyze();
            }

            pI2CProtocolCallback = I2C_Callback;
        }
        // Directly I2C command from host for I2C script mode
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_I2C)
        {
            uint8_t i;
            g_u16I2CTxDataLen = (pu8Buffer[HID_PREFIX_LEN + 2] << 8) | pu8Buffer[HID_PREFIX_LEN + 1]; // Exclude I2C adress
            g_u8SlaveAddr = pu8Buffer[HID_PREFIX_LEN];

            for (i = 0; i < g_sHidCmdH2D.u8DataLen; i++)
            {
                g_au8ToI2CDataBuf[i] = pu8Buffer[HID_PREFIX_LEN + 1 + i]; // I2C address does not store in the data buffer
            }

            pI2CProtocolCallback = I2C_Callback;
        }
        // ---------- I2C protocol end ----------

        // ---------- Codec protocol start ----------
        // Set I2S
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_I2S_SETTING)
        {
            printf("I2S Set\r\n");

            if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
            {
                switch (pu8Buffer[4]) // sample rate
                {
                    case 0:
                        g_u32I2S0SampleRate = 96000;
                        sUserConfig.u8SampleRate = SR_96kHz;
                        break;

                    case 1:
                        g_u32I2S0SampleRate = 48000;
                        sUserConfig.u8SampleRate = SR_48kHz;
                        break;

                    case 2:
                        g_u32I2S0SampleRate = 32000;
                        sUserConfig.u8SampleRate = SR_32kHz;
                        break;

                    case 3:
                        g_u32I2S0SampleRate = 16000;
                        sUserConfig.u8SampleRate = SR_16kHz;
                        break;

                    case 4:
                        g_u32I2S0SampleRate = 8000;
                        sUserConfig.u8SampleRate = SR_8kHz;
                        break;
                }

                switch (pu8Buffer[5])
                {
                    case 0:
                        g_u32I2S0Channel = I2S_TDMCHNUM_2CH;
                        sUserConfig.u8I2SChannelNum = I2S_2CHANNELS;
                        break;

                    case 1:
                        g_u32I2S0Channel = I2S_TDMCHNUM_4CH;
                        sUserConfig.u8I2SChannelNum = I2S_4CHANNELS;
                        break;
                }

                switch (pu8Buffer[6])
                {
                    case 0:
                        g_u32I2S0SerialMode = I2S_MODE_SLAVE;
                        sUserConfig.u8I2SDSync_SerialMode = I2S_Slave;
                        break;

                    case 1:
                        g_u32I2S0SerialMode = I2S_MODE_MASTER;
                        sUserConfig.u8I2SDSync_SerialMode = I2S_Master;
                        break;
                }

                switch (pu8Buffer[7])
                {
                    case 0:
                        g_u32I2S0Format = I2S_FORMAT_I2S;
                        sUserConfig.u8I2S0DInOut_Format = I2S_Standard;
                        break;

                    case 1:
                        g_u32I2S0Format = I2S_FORMAT_I2S_MSB;
                        sUserConfig.u8I2S0DInOut_Format = I2S_Left;
                        break;

                    case 2:
                        g_u32I2S0Format = I2S_FORMAT_PCM;
                        sUserConfig.u8I2S0DInOut_Format = PCM_Standard;
                        break;

                    case 3:
                        g_u32I2S0Format = I2S_FORMAT_PCM_MSB;
                        sUserConfig.u8I2S0DInOut_Format = PCM_Left;
                        break;
                }

                switch (pu8Buffer[8])
                {
                    case 0:
                        g_u32I2S0WordSize = I2S_DATABIT_16;
                        sUserConfig.u8I2S0DInOut_WordSize = I2S_16bit;
                        break;

                    case 1:
                        g_u32I2S0WordSize = I2S_DATABIT_32;
                        sUserConfig.u8I2S0DInOut_WordSize = I2S_32bit;
                        break;

                    case 2:
                        g_u32I2S0WordSize = I2S_DATABIT_24;
                        sUserConfig.u8I2S0DInOut_WordSize = I2S_24bit;
                        break;
                }

                switch (pu8Buffer[9])
                {
                    case 0:
                        g_u32I2S0FrameSize = I2S_CHWIDTH_16;
                        sUserConfig.u8I2S0DInOut_FrameSize = I2S_32Fs;
                        break;

                    case 1:
                        g_u32I2S0FrameSize = I2S_CHWIDTH_32;
                        sUserConfig.u8I2S0DInOut_FrameSize = I2S_64Fs;
                        break;
                }

                if ((pu8Buffer[10] == 1) || ((g_u32I2S0Channel == I2S_TDMCHNUM_4CH) || (g_u32I2S0Channel_old != g_u32I2S0Channel))) // execute FMC write
                {
                    g_u8ApplyFlag = 1; // Save I2S setting
                }

                if ((g_u32I2S0Channel == I2S_TDMCHNUM_4CH) && (g_u32I2S0SampleRate == 32000))
                {
                    g_u8UAC_ADAPTIVE = 1;
                }
                else
                {
                    g_u8UAC_ADAPTIVE = 0;
                }

                PlaybackBuffInit();
                RecordBuffInit();
                Switch_I2S_Sample_Rate();
            } // end if(g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)

            g_u8HIDCommandProcessing = FINISHED;
        }
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_Monitor_data)
        {
            if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
            {
                g_u8SlaveAddr = pu8Buffer[4];
                g_u8Channel_mode = pu8Buffer[5]; // Data command type
                g_u8RegisterAddrH = pu8Buffer[6];
                g_u8RegisterAddrL = pu8Buffer[7];
                g_u8Sample_Num_L = pu8Buffer[8]; // sample number low byte
                g_u8T1_delay_L = pu8Buffer[9];   // T1 delay Low Byte
            }

            g_u8HIDCommandProcessing = FINISHED;
            return;
        }
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_I2C_CLK_SET)
        {
            if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
            {

                if (pu8Buffer[4] == 1) // execute FMC write
                {
                    g_u8ApplyFlag = 1; // Save I2S setting
                }

                switch (pu8Buffer[5])
                {
                    case 0:
                        sUserConfig.u8I2C_ClockRate = I2C_Standard;
                        break;

                    case 1:
                        sUserConfig.u8I2C_ClockRate = I2C_Fast;
                        break;

                    case 2:
                        sUserConfig.u8I2C_ClockRate = I2C_Fast_Plus;
                        break;
                }
            }

            g_u8HIDCommandProcessing = FINISHED;
            return;
        }
        // transmit codec data - Burst mode
        else if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_9BIT ||
                 g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_8BIT ||
                 g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_16BIT)
        {
            // Copy data into ToI2C buffer
            u8FrameCnt = g_sHidCmdH2D.u8FrameNum & 0x7F;

            if (u8FrameCnt == 0)
            {
                g_u8DeviceID = pu8Buffer[4];
                g_u8RegisterAddrH = pu8Buffer[5];
                g_u8RegisterAddrL = pu8Buffer[6];
                g_u8BurstLen_H = pu8Buffer[7];
                g_u8BurstLen_L = pu8Buffer[8];

                if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    g_u16Data_Len = (g_u8BurstLen_H << 8) | g_u8BurstLen_L;
                else if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    g_u16Data_Len = g_sHidCmdH2D.u8DataLen - 5; // Deduct pu8Buffer[4],[5],[6],[7],[8]
            }

            switch (g_sHidCmdH2D.u8CommandType)
            {
                case HIDTRANS_CMD_CODEC_BURST_DATA_9BIT:
                    g_u8SlaveAddr = g_u8DeviceID; // pu8Buffer[4];

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    {
                        if (u8FrameCnt == 0)
                        {
                            g_u16I2CTxDataLen += g_u16Data_Len + 1; // add register address 1 byte
                            g_au8ToI2CDataBuf[0] = ((pu8Buffer[6] << 1) | (pu8Buffer[9] & 0x01));

                            for (i = 0; i < (g_u16Data_Len - 1); i++) // g_u16Data_Len-1 is to deduct pu8Buffer[9] this one
                            {
                                g_au8ToI2CDataBuf[i + 1] = pu8Buffer[i + 10];
                            }
                        }
                        else
                        {
                            g_u16Data_Len = g_sHidCmdH2D.u8DataLen;
                            HSUSBD_MemCopy((g_au8ToI2CDataBuf + (u8FrameCnt * 60) - 5), (pu8Buffer + HID_PREFIX_LEN), g_sHidCmdH2D.u8DataLen);
                            g_u16I2CTxDataLen += g_u16Data_Len; // register 1 byte and data 1 byte
                        }
                    }
                    else if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        g_u16I2CTxDataLen = 1;             // Write 1 byte register
                        g_u16I2CRxDataLen = g_u16Data_Len; // Read data length is 2
                        g_au8ToI2CDataBuf[0] = pu8Buffer[6] << 1;
                    }

                    break;

                case HIDTRANS_CMD_CODEC_BURST_DATA_8BIT:
                    g_u8SlaveAddr = g_u8DeviceID; // pu8Buffer[4];

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    {
                        if (u8FrameCnt == 0)
                        {
                            g_u16I2CTxDataLen += g_u16Data_Len + 1; // add register address 1 byte
                            g_au8ToI2CDataBuf[0] = pu8Buffer[6];

                            for (i = 0; i < g_u16Data_Len; i++)
                            {
                                g_au8ToI2CDataBuf[i + 1] = pu8Buffer[i + 9];
                            }
                        }
                        else
                        {
                            g_u16Data_Len = g_sHidCmdH2D.u8DataLen;
                            HSUSBD_MemCopy((g_au8ToI2CDataBuf + (u8FrameCnt * 60) - 4), (pu8Buffer + HID_PREFIX_LEN), g_sHidCmdH2D.u8DataLen);
                            g_u16I2CTxDataLen += g_u16Data_Len; // 2; register 1 byte and data 1 byte
                        }
                    }
                    else if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        g_u16I2CTxDataLen = 1;             // Write 1 byte register
                        g_u16I2CRxDataLen = g_u16Data_Len; // Read data length is 1
                        g_au8ToI2CDataBuf[0] = pu8Buffer[6];
                    }

                    break;

                case HIDTRANS_CMD_CODEC_BURST_DATA_16BIT:
                    g_u8SlaveAddr = g_u8DeviceID; // pu8Buffer[4];

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    {
                        if (u8FrameCnt == 0)
                        {
                            g_u16I2CTxDataLen += g_u16Data_Len + 2; // add register address 2 byte
                            g_au8ToI2CDataBuf[0] = pu8Buffer[5];
                            g_au8ToI2CDataBuf[1] = pu8Buffer[6];

                            for (i = 0; i < g_u16Data_Len; i++)
                            {
                                g_au8ToI2CDataBuf[i + 2] = pu8Buffer[i + 9];
                            }
                        }
                        else
                        {
                            g_u16Data_Len = g_sHidCmdH2D.u8DataLen;
                            HSUSBD_MemCopy((g_au8ToI2CDataBuf + (u8FrameCnt * 60) - 3), (pu8Buffer + HID_PREFIX_LEN), g_sHidCmdH2D.u8DataLen);
                            g_u16I2CTxDataLen += g_u16Data_Len; // register 1 byte and data 1 byte
                        }
                    }
                    else if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        g_u16I2CTxDataLen = 2;             // Write 2 byte register
                        g_u16I2CRxDataLen = g_u16Data_Len; // Read data length is 2
                        g_au8ToI2CDataBuf[0] = pu8Buffer[5];
                        g_au8ToI2CDataBuf[1] = pu8Buffer[6];
                    }

                    break;
            }

            if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
            {
                if (g_sHidCmdH2D.u8FrameNum & 0x80) // The command has more than 60 data
                {

                    g_u8HIDCommandProcessing = WAITING_MORE_DATA;
                    return;
                }
                else
                {
                    g_u8HIDCommandProcessing = PROCESSING;
                }
            }

            pI2CProtocolCallback = Codec_I2CCallback;
        }
        else // transmit codec data - Normal mode
        {
            g_u8DeviceID = pu8Buffer[4];
            g_u8RegisterAddrH = pu8Buffer[5];
            g_u8RegisterAddrL = pu8Buffer[6];

            if (g_sHidCmdH2D.u8DataLen < 4)
            {
                if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_DATA_8BIT)
                    g_u16Data_Len = 1;
                else
                    g_u16Data_Len = 2;
            }
            else if (g_sHidCmdH2D.u8DataLen == 4)
            {
                g_u16Data_Len = pu8Buffer[7];
            }
            else
            {
                if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_DATA_16BIT)
                    g_u16Data_Len = g_sHidCmdH2D.u8DataLen - 2; // Deduct pu8Buffer[4] & pu8Buffer[7]
                else
                    g_u16Data_Len = g_sHidCmdH2D.u8DataLen - 4; // Deduct pu8Buffer[4],[5],[6],[7]
            }

            switch (g_sHidCmdH2D.u8CommandType)
            {
                case HIDTRANS_CMD_CODEC_DATA_9BIT:
                    g_u8SlaveAddr = g_u8DeviceID; // pu8Buffer[4];

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    {
                        g_u16I2CTxDataLen = g_u16Data_Len; // register 1 byte and data 1 byte
                        g_au8ToI2CDataBuf[0] = ((pu8Buffer[6] << 1) | (pu8Buffer[8] & 0x01));

                        for (i = 0; i < (g_u16Data_Len - 1); i++) // g_u16Data_Len-1 is to deduct pu8Buffer[8] this one
                        {
                            g_au8ToI2CDataBuf[i + 1] = pu8Buffer[i + 9];
                        }
                    }
                    else if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        g_u16I2CTxDataLen = 1;             // Write 1 byte register
                        g_u16I2CRxDataLen = g_u16Data_Len; // Read data length is 2
                        g_au8ToI2CDataBuf[0] = pu8Buffer[6] << 1;
                    }

                    break;

                case HIDTRANS_CMD_CODEC_DATA_8BIT:
                    g_u8SlaveAddr = g_u8DeviceID; // pu8Buffer[4];

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    {
                        if (g_u16Data_Len > 2)
                        {
                            g_u16I2CTxDataLen = g_u16Data_Len; // 2; // register 1 byte and data 1 byte
                            g_au8ToI2CDataBuf[0] = pu8Buffer[6];

                            for (i = 0; i < g_u16Data_Len; i++)
                            {
                                g_au8ToI2CDataBuf[i + 1] = pu8Buffer[i + 8];
                            }
                        }
                        else
                        {
                            g_u16I2CTxDataLen = 2; // register 1 byte and data 1 byte
                            g_au8ToI2CDataBuf[0] = pu8Buffer[6];
                            g_au8ToI2CDataBuf[1] = pu8Buffer[9];
                        }
                    }
                    else if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        g_u16I2CTxDataLen = 1;             // Write 1 byte register
                        g_u16I2CRxDataLen = g_u16Data_Len; // Read data length is 1
                        g_au8ToI2CDataBuf[0] = pu8Buffer[6];
                    }

                    break;

                case HIDTRANS_CMD_CODEC_DATA_16BIT:
                    g_u8SlaveAddr = g_u8DeviceID; // pu8Buffer[4];

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_WRITE)
                    {
                        g_u16I2CTxDataLen = g_u16Data_Len; // register 1 byte and data 1 byte
                        g_au8ToI2CDataBuf[0] = pu8Buffer[5];
                        g_au8ToI2CDataBuf[1] = pu8Buffer[6];

                        for (i = 0; i < g_u16Data_Len; i++)
                        {
                            g_au8ToI2CDataBuf[i + 2] = pu8Buffer[i + 8];
                        }
                    }
                    else if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        g_u16I2CTxDataLen = 2;             // Write 2 byte register
                        g_u16I2CRxDataLen = g_u16Data_Len; // Read data length is 2
                        g_au8ToI2CDataBuf[0] = pu8Buffer[5];
                        g_au8ToI2CDataBuf[1] = pu8Buffer[6];
                    }

                    break;
            }

            pI2CProtocolCallback = Codec_I2CCallback;
        }

        // ---------- Codec protocol end ----------

        // Start I2C transmission
        I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA);
    }
    else if (g_u8HIDCommandProcessing == PROCESSING)
    {
        // During data processing, reply the connection result directly
        if (pu8Buffer[0] == HIDTRANS_CHECK)
        {
            g_u8CheckConnection = 2;
        }
    }
}

/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void HSUSBD_IRQHandler(void)
{
    __IO uint32_t IrqStL, IrqSt;
    volatile uint8_t u8Buf[8];

    IrqStL = HSUSBD->GINTSTS & HSUSBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)    return;

    HSUSBD_MemCopy((uint8_t *)u8Buf, (uint8_t *)&gUsbCmd, 8);

    /* USB interrupt */
    if (IrqStL & HSUSBD_GINTSTS_USBIF_Msk)
    {
        IrqSt = HSUSBD->BUSINTSTS & HSUSBD->BUSINTEN;

        if (IrqSt & HSUSBD_BUSINTSTS_SOFIF_Msk)
        {
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SOFIF_Msk);

            HIRC_AutoTrim_Enable();

            if (g_u32SpeakerByteCountRec != g_u32SpeakerByteCount)
            {
                g_u32SpeakerByteCountRec = g_u32SpeakerByteCount;
            }
            else
            {
                g_u8AudioSpeakerDebounce++;

                if (g_u8AudioSpeakerDebounce >= 5)
                {
                    g_u8AudioSpeakerDebounce = 0;
                    g_u32SpeakerByteCount = 0;
                    g_u32SpeakerByteCountRec = 0;
                }
            }
        }

        if (IrqSt & HSUSBD_BUSINTSTS_RSTIF_Msk)
        {
            HSUSBD_SwReset();
            HSUSBD_ResetDMA();
            HIRC_AutoTrim_Reset();

            HSUSBD->EP[EPG].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;
            HSUSBD->EP[EPH].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;

            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_SET_ADDR(0);
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RSTIF_Msk);
            HSUSBD_CLR_CEP_INT_FLAG(0x1ffc);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_RESUMEIF_Msk)
        {
            HIRC_AutoTrim_Reset();

            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_SUSPENDIF_Msk)
        {
            HIRC_AutoTrim_Reset();

            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SUSPENDIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_HISPDIF_Msk)
        {
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_HISPDIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_DMADONEIF_Msk)
        {
            g_hsusbd_DmaDone = 1;
            printf("Read command - Complete\n");
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_DMADONEIF_Msk);

            if (HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk)
            {
                g_u8HIDInDone = 1;

                if (g_hsusbd_ShortPacket == 1)
                {
                    HSUSBD->EP[EPA].EPRSPCTL = (HSUSBD->EP[EPA].EPRSPCTL & 0x10) | HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
                    g_hsusbd_ShortPacket = 0;
                }
            }
        }

        if (IrqSt & HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if (IrqSt & HSUSBD_BUSINTSTS_VBUSDETIF_Msk)
        {
            if (HSUSBD_IS_ATTACHED())
            {
                /* USB Plug In */
                HSUSBD_ENABLE_USB();
                HSUSBD_ENABLE_HS_HANDSHAKE();
            }
            else
            {
                /* USB Un-plug */
                HSUSBD_DISABLE_USB();
                HSUSBD_DISABLE_HS_HANDSHAKE();
            }

            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    if (IrqStL & HSUSBD_GINTSTS_CEPIF_Msk)
    {
        IrqSt = HSUSBD->CEPINTSTS & HSUSBD->CEPINTEN;

        if (IrqSt & HSUSBD_CEPINTSTS_SETUPTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPTKIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_SETUPPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPPKIF_Msk);
            HSUSBD_ProcessSetupPacket();
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_OUTTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_OUTTKIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);

            HSUSBD_MemCopy((uint8_t *)u8Buf, (uint8_t *)&gUsbCmd, 8);

            if (u8Buf[0] == 0x22)
            {
                if ((g8_Board_mode != Board_Normal))
                {
                    if (g_usbd_PlaySampleRate != g_u32I2S0SampleRate_old)
                    {
                        g_u32I2S0SampleRate = g_usbd_PlaySampleRate;
                        PlaybackBuffInit();
                        RecordBuffInit();
                        Switch_I2S_Sample_Rate();

                        switch (g_usbd_PlaySampleRate) // save sample rate
                        {
                            case 96000:
                                sUserConfig.u8SampleRate = SR_96kHz;
                                break;

                            case 48000:
                                sUserConfig.u8SampleRate = SR_48kHz;
                                break;

                            case 32000:
                                sUserConfig.u8SampleRate = SR_32kHz;
                                break;

                            case 16000:
                                sUserConfig.u8SampleRate = SR_16kHz;
                                break;

                            case 8000:
                                sUserConfig.u8SampleRate = SR_8kHz;
                                break;
                        }

                        Flash_Memory(); // Save data to data flash first and then send Ack to PC
                        DelayUs(50);
                    }

                    g_u32I2S0SampleRate_old = g_usbd_PlaySampleRate;
                }

                if (u8Buf[3] == SAMPLING_FREQ_CONTROL && (u8Buf[4] == (EPD | EP_OUTPUT)))
                {
                    g_u8USBPlayEn = 1;
                }
                else if (u8Buf[3] == SAMPLING_FREQ_CONTROL && (u8Buf[4] == (EPC | EP_INPUT)))
                {
                    if (g8_Board_mode != Board_Special) // Because special mode not recorder function
                        g_u8USBRecEn = 1;
                }
            }

            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_INTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);

            if (!(IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk))
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_TXPKIEN_Msk);
                HSUSBD_CtrlIn();
            }
            else
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_TXPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }

            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_PINGIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_PINGIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_TXPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);

            if (g_hsusbd_CtrlInSize)
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
            }
            else
            {
                if (g_hsusbd_CtrlZero == 1)
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_ZEROLEN);

                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }

            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_RXPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_RXPKIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_NAKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_NAKIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_STALLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STALLIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_ERRIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_ERRIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk)
        {
            HSUSBD_UpdateDeviceState();
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_BUFFULLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFFULLIF_Msk);
            return;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk);
            return;
        }
    }

    if (IrqStL & HSUSBD_GINTSTS_EPAIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPA].EPINTSTS & HSUSBD->EP[EPA].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPBIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPB].EPINTSTS & HSUSBD->EP[EPB].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPB, IrqSt);
    }

    /* isochronous in */
    if (IrqStL & HSUSBD_GINTSTS_EPCIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPC].EPINTSTS & HSUSBD->EP[EPC].EPINTEN;
        EPC_Handler();
        HSUSBD_CLR_EP_INT_FLAG(EPC, IrqSt);
    }

    /* isochronous out */
    if (IrqStL & HSUSBD_GINTSTS_EPDIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPD].EPINTSTS & HSUSBD->EP[EPD].EPINTEN;
        EPD_Handler();
        HSUSBD_CLR_EP_INT_FLAG(EPD, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPEIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPE].EPINTSTS & HSUSBD->EP[EPE].EPINTEN;
        EPE_Handler();
        HSUSBD_CLR_EP_INT_FLAG(EPE, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPFIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPF].EPINTSTS & HSUSBD->EP[EPF].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPF, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPGIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPG].EPINTSTS & HSUSBD->EP[EPG].EPINTEN;
        EPG_Handler();
        HSUSBD_CLR_EP_INT_FLAG(EPG, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPHIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPH].EPINTSTS & HSUSBD->EP[EPH].EPINTEN;

        //if (HSUSBD->EP[EPH].EPINTSTS & 0x01)
        if (HSUSBD->EP[EPH].EPINTSTS & HSUSBD_EPINTSTS_RXPKIF_Msk)
            EPH_Handler();

        HSUSBD_CLR_EP_INT_FLAG(EPH, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPIIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPI].EPINTSTS & HSUSBD->EP[EPI].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPI, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPJIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPJ].EPINTSTS & HSUSBD->EP[EPJ].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPJ, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPKIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPK].EPINTSTS & HSUSBD->EP[EPK].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPK, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPLIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPL].EPINTSTS & HSUSBD->EP[EPL].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPL, IrqSt);
    }
}

/*--------------------------------------------------------------------------*/

uint16_t g_u16RecPtrDiff = 0;
// ISO IN handler
void EPC_Handler(void)
{
    int i;
    uint16_t u16distance = 0;
    uint16_t u16DataLen = 0;
    uint16_t u16EPC_Byte_Count = 0;
    uint16_t USB_BUFFER_EPC_BASE = 0;
    uint16_t USB_BUFFER_EPC_ADD = 0;
    uint16_t USB_BUFFER_EPC_SUB = 0;
    uint8_t *pu8buf = NULL;

    if (g_u8USBTxDataLen == USB_16bit)
    {
        if (g_u32I2S0Channel == I2S_TDMCHNUM_4CH)
        {
            USB_BUFFER_EPC_BASE = g_u32I2S0SampleRate * REC_4CHANNELS * 2 / 1000; // 48k * 2ch *2 byte
            USB_BUFFER_EPC_ADD = 2 * REC_4CHANNELS * 2;
            USB_BUFFER_EPC_SUB = 2 * REC_4CHANNELS * 2;
        }
        else
        {
            USB_BUFFER_EPC_BASE = g_u32I2S0SampleRate * REC_CHANNELS * 2 / 1000; // 48k * 2ch *2 byte
            USB_BUFFER_EPC_ADD = 2 * REC_CHANNELS * 2;
            USB_BUFFER_EPC_SUB = 2 * REC_CHANNELS * 2;
        }
    }
    else if (g_u8USBTxDataLen == USB_24bit)
    {
        USB_BUFFER_EPC_BASE = g_u32I2S0SampleRate * REC_CHANNELS * 2 / 1000; // 48k * 2ch *3 byte
        USB_BUFFER_EPC_ADD = 2 * REC_CHANNELS * 3;
        USB_BUFFER_EPC_SUB = 2 * REC_CHANNELS * 3;
    }

    if (g_u8AudioRecorderState == UAC_BUSY_AUDIO_RECORD)
    {
        // This section calculate the data number in UAC_RingBuffer.
        // And set the next transfer size depends on buffer threshold.
        if (g_u16Record_Write_Ptr >= g_u16Record_Read_Ptr)
        {
            u16distance = g_u16Record_Write_Ptr - g_u16Record_Read_Ptr;
        }
        else
        {
            u16distance = (g_u16UAC_MAX_USB_BUFFER_LEN - g_u16Record_Read_Ptr) + g_u16Record_Write_Ptr;
        }

        g_u16RecPtrDiff = u16distance;

        if (u16distance >= g_u16UAC_USB_BUFF_UPPER_THRE)
        {
            u16EPC_Byte_Count = USB_BUFFER_EPC_BASE + USB_BUFFER_EPC_ADD;
        }
        else if (u16distance < g_u16UAC_USB_BUFF_LOWER_THRE)
        {
            u16EPC_Byte_Count = USB_BUFFER_EPC_BASE - USB_BUFFER_EPC_SUB;
        }
        else
        {
            u16EPC_Byte_Count = USB_BUFFER_EPC_BASE;
        }

        pu8buf = (uint8_t *)g_au8IsoInBuff;

        u16DataLen = u16EPC_Byte_Count >> 1; // 16-bit data

        if (g_usbd_RecMute)
        {
            for (i = 0; i < u16DataLen; i++)
            {
                pu8buf[i * 2] = 0;
                pu8buf[i * 2 + 1] = 0;
            }
        }
        else
        {
            if ((I2S_PORT->CTL0 & I2S_CTL0_TDMCHNUM_Msk) == I2S_TDMCHNUM_2CH)
            {
                for (i = 0; i < u16DataLen; i++)
                {
                    if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_16)
                    {
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 0) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 8) & 0xff;
                        g_u16Record_Read_Ptr++;

                        if (g_u16Record_Read_Ptr >= g_u16UAC_MAX_USB_BUFFER_LEN)
                        {
                            g_u16Record_Read_Ptr = 0;
                        }
                    }
                    else if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_24)
                    {
#if (I2S_FIFO_24BIT_LSB)
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 8) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 16) & 0xff;
#elif (I2S_FIFO_24BIT_MSB)
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 16) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 24) & 0xff;
#endif
                        g_u16Record_Read_Ptr++;

                        if (g_u16Record_Read_Ptr >= g_u16UAC_MAX_USB_BUFFER_LEN)
                        {
                            g_u16Record_Read_Ptr = 0;
                        }
                    }
                    else // ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_32)
                    {
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 16) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 24) & 0xff;

                        g_u16Record_Read_Ptr++;

                        if (g_u16Record_Read_Ptr >= g_u16UAC_MAX_USB_BUFFER_LEN)
                        {
                            g_u16Record_Read_Ptr = 0;
                        }
                    }
                }
            }
            else
            {
                for (i = 0; i < u16DataLen; i++)
                {
                    if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_16)
                    {
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 0) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 8) & 0xff;
                        g_u16Record_Read_Ptr++;

                        if (g_u16Record_Read_Ptr >= g_u16UAC_MAX_USB_BUFFER_LEN)
                        {
                            g_u16Record_Read_Ptr = 0;
                        }
                    }
                    else if ((I2S_PORT->CTL0 & I2S_CTL0_DATWIDTH_Msk) == I2S_DATABIT_24)
                    {
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 8) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 16) & 0xff;
                        g_u16Record_Read_Ptr++;

                        if (g_u16Record_Read_Ptr >= g_u16UAC_MAX_USB_BUFFER_LEN)
                        {
                            g_u16Record_Read_Ptr = 0;
                        }
                    }
                    else
                    {
                        pu8buf[i * 2] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 16) & 0xff;
                        pu8buf[i * 2 + 1] = (g_au32USBRecRingBuff[g_u16Record_Read_Ptr] >> 24) & 0xff;
                        g_u16Record_Read_Ptr++;

                        if (g_u16Record_Read_Ptr >= g_u16UAC_MAX_USB_BUFFER_LEN)
                        {
                            g_u16Record_Read_Ptr = 0;
                        }
                    }
                }
            }
        }

        for (i = 0; i < u16EPC_Byte_Count; i++)
        {
            HSUSBD->EP[EPC].EPDAT_BYTE = pu8buf[i];
        }

        HSUSBD->EP[EPC].EPTXCNT = u16EPC_Byte_Count;
        HSUSBD_ENABLE_EP_INT(EPC, HSUSBD_EPINTEN_INTKIEN_Msk);
        memset((void *)g_au8IsoInBuff, 0, ((u16EPC_Byte_Count > EPC_MAX_PKT_SIZE) ? EPC_MAX_PKT_SIZE : u16EPC_Byte_Count)); // clear buffer
    }
    else
    {
        HSUSBD->EP[EPC].EPRSPCTL = HSUSBD_EPRSPCTL_ZEROLEN_Msk;
    }

    return;
}

volatile uint32_t u32CheckBufferTimerCount = 0;
// ISO OUT handler
void EPD_Handler(void)
{
    uint32_t i;
    uint8_t *pu8Buf;
    uint16_t *pu16Buf;
    uint16_t u16Outlen;
    uint32_t *pu32RingBuff = (uint32_t *)&g_au32USBPlayRingBuff[0];

    if (g_u8USBTxDataLen == USB_24bit)
    {
        // Get payload of EPD.
        g_u16PayLoadLen = HSUSBD->EP[EPD].EPDATCNT & 0xffff; // 2ch * 48 * 2byte
        g_u32SpeakerByteCount += g_u16PayLoadLen;

        for (i = 0; i < g_u16PayLoadLen; i++)
            g_au8IsoOutBuff[i] = HSUSBD->EP[EPD].EPDAT_BYTE;

        if (g_u8AudioSpeakerState == UAC_BUSY_AUDIO_SPEAK)
        {
            pu8Buf = (uint8_t *)g_au8IsoOutBuff;
            u16Outlen = g_u16PayLoadLen / 12; // 2ch * 48samples

            if ((g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr) && (g_bUnderFlow == FALSE))
            {
                g_bOverFlow = TRUE;
            }
            else
            {
                g_bUnderFlow = FALSE;

                for (i = 0; i < u16Outlen; i++)
                {
                    // Get data from End-Point buffer.
                    pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i] | (pu8Buf[12 * i + 1] << 8) | (pu8Buf[12 * i + 2] << 16);       // left
                    pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 3] | (pu8Buf[12 * i + 4] << 8) | (pu8Buf[12 * i + 5] << 16);   // right
                    pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 6] | (pu8Buf[12 * i + 7] << 8) | (pu8Buf[12 * i + 8] << 16);   // left
                    pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 9] | (pu8Buf[12 * i + 10] << 8) | (pu8Buf[12 * i + 11] << 16); // right

                    // Ring buffer write pointer turn around.
                    if (g_u16PlayBack_Write_Ptr >= g_u16PlayBack_MAX_USB_BUFFER_LEN)
                        g_u16PlayBack_Write_Ptr = 0;
                }
            }

            if (g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr)
                g_bOverFlow = TRUE;

            if (u32CheckBufferTimerCount++ == 5)
            {
                Adjust_PLL();
                u32CheckBufferTimerCount = 0;
            }

            if (!g_u8USBPlayEn)
            {
                if (g_u16PlayBack_Write_Ptr >= (g_u16PlayBack_MAX_USB_BUFFER_LEN >> 1))
                {
                    g_u8USBPlayEn = 1;
                }
            }
        }
        // Audio start. To setup Buffer Control.
        else if (g_u8AudioSpeakerState == UAC_START_AUDIO_SPEAK)
        {
            pu8Buf = (uint8_t *)g_au8IsoOutBuff;
            g_u8AudioSpeakerState = UAC_PROCESS1_AUDIO_SPEAK;
            g_u16PlayBack_Write_Ptr = 0;
            g_u16PlayBack_Read_Ptr = 0;

            u16Outlen = g_u16PayLoadLen / 12;

            for (i = 0; i < u16Outlen; i++)
            {
                // Get data from End-Point buffer.
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i] | (pu8Buf[12 * i + 1] << 8) | (pu8Buf[12 * i + 2] << 16);       // left
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 3] | (pu8Buf[12 * i + 4] << 8) | (pu8Buf[12 * i + 5] << 16);   // right
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 6] | (pu8Buf[12 * i + 7] << 8) | (pu8Buf[12 * i + 8] << 16);   // left
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 9] | (pu8Buf[12 * i + 10] << 8) | (pu8Buf[12 * i + 11] << 16); // right
            }
        }
        // First Audio transfer.
        else if (g_u8AudioSpeakerState == UAC_PROCESS1_AUDIO_SPEAK)
        {
            pu8Buf = (uint8_t *)g_au8IsoOutBuff;

            g_u8AudioSpeakerState = UAC_PROCESS2_AUDIO_SPEAK;

            u16Outlen = g_u16PayLoadLen / 12;

            for (i = 0; i < u16Outlen; i++)
            {
                // Get data from End-Point buffer.
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i] | (pu8Buf[12 * i + 1] << 8) | (pu8Buf[12 * i + 2] << 16);       // left
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 3] | (pu8Buf[12 * i + 4] << 8) | (pu8Buf[12 * i + 5] << 16);   // right
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 6] | (pu8Buf[12 * i + 7] << 8) | (pu8Buf[12 * i + 8] << 16);   // left
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 9] | (pu8Buf[12 * i + 10] << 8) | (pu8Buf[12 * i + 11] << 16); // right
            }
        }
        // Second Audio transfer.
        else if (g_u8AudioSpeakerState == UAC_PROCESS2_AUDIO_SPEAK)
        {
            pu8Buf = (uint8_t *)g_au8IsoOutBuff;

            g_u8AudioSpeakerState = UAC_BUSY_AUDIO_SPEAK;

            u16Outlen = g_u16PayLoadLen / 12;

            for (i = 0; i < u16Outlen; i++)
            {
                // Get data from End-Point buffer.
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i] | (pu8Buf[12 * i + 1] << 8) | (pu8Buf[12 * i + 2] << 16);       // left
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 3] | (pu8Buf[12 * i + 4] << 8) | (pu8Buf[12 * i + 5] << 16);   // right
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 6] | (pu8Buf[12 * i + 7] << 8) | (pu8Buf[12 * i + 8] << 16);   // left
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu8Buf[12 * i + 9] | (pu8Buf[12 * i + 10] << 8) | (pu8Buf[12 * i + 11] << 16); // right
            }

            I2S_ENABLE_TX(I2S_PORT);
        }

        /* Prepare for nex OUT packet */
        HSUSBD_SET_MAX_PAYLOAD(EPD, EPD_MAX_PKT_SIZE_24bit);
    }
    else if (g_u8USBTxDataLen == USB_16bit)
    {
        // Get payload of EPD.
        g_u16PayLoadLen = HSUSBD->EP[EPD].EPDATCNT & 0xffff; // 2ch * 48 * 2byte
        g_u32SpeakerByteCount += g_u16PayLoadLen;

        for (i = 0; i < g_u16PayLoadLen; i++)
            g_au8IsoOutBuff[i] = HSUSBD->EP[EPD].EPDAT_BYTE;

        if (g_u8AudioSpeakerState == UAC_BUSY_AUDIO_SPEAK)
        {
            pu16Buf = (uint16_t *)g_au8IsoOutBuff;

            u16Outlen = g_u16PayLoadLen >> 1; // 2ch * 48samples

            if ((g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr) && (g_bUnderFlow == FALSE))
            {
                g_bOverFlow = TRUE;
            }
            else
            {
                g_bUnderFlow = FALSE;

                for (i = 0; i < u16Outlen; i++)
                {
                    // Get data from End-Point buffer.
                    pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu16Buf[i];

                    // Ring buffer write pointer turn around.
                    if (g_u16PlayBack_Write_Ptr >= g_u16PlayBack_MAX_USB_BUFFER_LEN)
                        g_u16PlayBack_Write_Ptr = 0;
                }
            }

            if (g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr)
                g_bOverFlow = TRUE;

            if (u32CheckBufferTimerCount++ == 10)
            {
                Adjust_PLL();
                u32CheckBufferTimerCount = 0;
            }

            if (!g_u8USBPlayEn)
            {
                if (g_u16PlayBack_Write_Ptr >= (g_u16PlayBack_MAX_USB_BUFFER_LEN >> 1))
                {
                    g_u8USBPlayEn = 1;
                }
            }
        }
        // Audio start. To setup Buffer Control.
        else if (g_u8AudioSpeakerState == UAC_START_AUDIO_SPEAK)
        {
            pu16Buf = (uint16_t *)g_au8IsoOutBuff;
            g_u8AudioSpeakerState = UAC_PROCESS1_AUDIO_SPEAK;
            g_u16PlayBack_Write_Ptr = 0;
            g_u16PlayBack_Read_Ptr = 0;

            u16Outlen = g_u16PayLoadLen >> 1;

            for (i = 0; i < u16Outlen; i++)
            {
                // Get data from End-Point buffer.
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu16Buf[i];
            }
        }
        // First Audio transfer.
        else if (g_u8AudioSpeakerState == UAC_PROCESS1_AUDIO_SPEAK)
        {
            pu16Buf = (uint16_t *)g_au8IsoOutBuff;

            g_u8AudioSpeakerState = UAC_PROCESS2_AUDIO_SPEAK;

            u16Outlen = g_u16PayLoadLen >> 1;

            for (i = 0; i < u16Outlen; i++)
            {
                // Get data from End-Point buffer.
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu16Buf[i];
            }
        }
        // Second Audio transfer.
        else if (g_u8AudioSpeakerState == UAC_PROCESS2_AUDIO_SPEAK)
        {
            pu16Buf = (uint16_t *)g_au8IsoOutBuff;

            g_u8AudioSpeakerState = UAC_BUSY_AUDIO_SPEAK;

            u16Outlen = g_u16PayLoadLen >> 1;

            for (i = 0; i < u16Outlen; i++)
            {
                // Get data from End-Point buffer.
                pu32RingBuff[g_u16PlayBack_Write_Ptr++] = pu16Buf[i];
            }

            I2S_ENABLE_TX(I2S_PORT);
        }

        /* Prepare for nex OUT packet */
        HSUSBD_SET_MAX_PAYLOAD(EPD, EPD_MAX_PKT_SIZE);
    }
}

// ISO IN Sync handler
void EPE_Handler(void)
{
}

void EPF_Handler(void)
{
}

// Interrupt IN handler
void EPG_Handler(void)
{
    g_u8HIDInDone = 1;
    HSUSBD_ENABLE_EP_INT(EPG, 0);
}

// Interrupt OUT handler
void EPH_Handler(void)
{
    uint32_t len, i;

    len = HSUSBD->EP[EPH].EPDATCNT & 0xffff;

    for (i = 0; i < len; i++)
        g_au8IntOutBuff[i] = HSUSBD->EP[EPH].EPDAT_BYTE;

    HIDTrans_ProcessCommand((uint8_t *)g_au8IntOutBuff, len);
}

/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class.
 *              HSUSBD is set to full speed mode.
 */
void UAC_Init(void)
{
    int32_t i;
    uint8_t *pu8;
    uint8_t *pSerial = (uint8_t *)&__TIME__;

    /* Configure USB controller */
    HSUSBD->OPER = 0; /* Full Speed */
    /* Enable USB BUS, CEP, EPC ,EPD, EPG and EPH global interrupt */
    HSUSBD_ENABLE_USB_INT(HSUSBD_GINTEN_USBIEN_Msk | HSUSBD_GINTEN_CEPIEN_Msk |
                          HSUSBD_GINTEN_EPCIEN_Msk | HSUSBD_GINTEN_EPDIEN_Msk |
                          HSUSBD_GINTEN_EPEIEN_Msk |
                          HSUSBD_GINTEN_EPGIEN_Msk | HSUSBD_GINTEN_EPHIEN_Msk);
    /* Enable BUS interrupt */
    HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    HSUSBD_SET_ADDR(SETUP_BUF_BASE);

    /*****************************************************/
    /* Control endpoint */
    HSUSBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);

    if (g8_Board_mode == Board_Special)
    {
        /*****************************************************/
        /* EPC ==> ISO IN endpoint, address 2 */
        HSUSBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN_24bit);
        HSUSBD_SET_MAX_PAYLOAD(EPC, EPC_MAX_PKT_SIZE_24bit);
        HSUSBD_ConfigEp(EPC, EPC, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_IN);
        HSUSBD_ENABLE_EP_INT(EPC, HSUSBD_EPINTEN_INTKIEN_Msk);

        /* EPD ==> ISO OUT endpoint, address 3 */
        HSUSBD_SetEpBufAddr(EPD, EPD_BUF_BASE_24bit, EPD_BUF_LEN_24bit);
        HSUSBD_SET_MAX_PAYLOAD(EPD, EPD_MAX_PKT_SIZE_24bit);
        HSUSBD_ConfigEp(EPD, EPD, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_OUT);
        HSUSBD_ENABLE_EP_INT(EPD, HSUSBD_EPINTEN_RXPKIEN_Msk | HSUSBD_EPINTEN_SHORTRXIEN_Msk);

        /*****************************************************/
        /* EPE ==> ISO IN endpoint, address 4 */
        HSUSBD_SetEpBufAddr(EPE, EPE_BUF_BASE_24bit, EPE_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPE, EPE_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPE, EPE, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_IN);
        HSUSBD_ENABLE_EP_INT(EPE, HSUSBD_EPINTEN_INTKIEN_Msk);

        /* EPF ==> ISO IN endpoint, address 5 */
        HSUSBD_SetEpBufAddr(EPF, EPF_BUF_BASE_24bit, EPF_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPF, EPF_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPF, EPF, 0, HSUSBD_EP_CFG_DIR_IN);
        HSUSBD_ENABLE_EP_INT(EPF, HSUSBD_EPINTEN_INTKIEN_Msk);

        /*****************************************************/
        /* EPG ==> INT IN endpoint, address 6 */
        HSUSBD_SetEpBufAddr(EPG, EPG_BUF_BASE_24bit, EPG_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPG, EPG_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPG, INT_IN_EP_NUM, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_IN);

        /* EPH ==> INT OUT endpoint, address 7 */
        HSUSBD_SetEpBufAddr(EPH, EPH_BUF_BASE_24bit, EPH_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPH, EPH_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPH, INT_OUT_EP_NUM, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_OUT);
        //HSUSBD_ENABLE_EP_INT(EPH, HSUSBD_EPINTEN_RXPKIEN_Msk |
        //HSUSBD_EPINTEN_BUFFULLIEN_Msk |
        //HSUSBD_EPINTEN_SHORTRXIEN_Msk);
        HSUSBD_ENABLE_EP_INT(EPH, HSUSBD_EPINTEN_RXPKIEN_Msk | HSUSBD_EPINTEN_BUFFULLIEN_Msk |
                             HSUSBD_EPINTEN_SHORTRXIEN_Msk | HSUSBD_EPINTEN_OUTTKIEN_Msk);
    }
    else
    {
        /*****************************************************/
        /* EPC ==> ISO IN endpoint, address 2 */
        HSUSBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPC, EPC_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPC, EPC, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_IN);
        HSUSBD_ENABLE_EP_INT(EPC, HSUSBD_EPINTEN_INTKIEN_Msk);

        /* EPD ==> ISO OUT endpoint, address 3 */
        HSUSBD_SetEpBufAddr(EPD, EPD_BUF_BASE, EPD_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPD, EPD_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPD, EPD, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_OUT);
        HSUSBD_ENABLE_EP_INT(EPD, HSUSBD_EPINTEN_RXPKIEN_Msk | HSUSBD_EPINTEN_SHORTRXIEN_Msk);

        /*****************************************************/
        /* EPE ==> ISO IN endpoint, address 4 */
        HSUSBD_SetEpBufAddr(EPE, EPE_BUF_BASE, EPE_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPE, EPE_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPE, EPE, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_IN);
        HSUSBD_ENABLE_EP_INT(EPE, HSUSBD_EPINTEN_INTKIEN_Msk);

        /* EPF ==> ISO IN endpoint, address 5 */
        HSUSBD_SetEpBufAddr(EPF, EPF_BUF_BASE, EPF_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPF, EPF_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPF, EPF, 0, HSUSBD_EP_CFG_DIR_IN);
        HSUSBD_ENABLE_EP_INT(EPF, HSUSBD_EPINTEN_INTKIEN_Msk);

        /*****************************************************/
        /* EPG ==> INT IN endpoint, address 6 */
        HSUSBD_SetEpBufAddr(EPG, EPG_BUF_BASE, EPG_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPG, EPG_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPG, INT_IN_EP_NUM, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_IN);

        /* EPH ==> INT OUT endpoint, address 7 */
        HSUSBD_SetEpBufAddr(EPH, EPH_BUF_BASE, EPH_BUF_LEN);
        HSUSBD_SET_MAX_PAYLOAD(EPH, EPH_MAX_PKT_SIZE);
        HSUSBD_ConfigEp(EPH, INT_OUT_EP_NUM, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_OUT);
        HSUSBD_ENABLE_EP_INT(EPH, HSUSBD_EPINTEN_RXPKIEN_Msk |
                             HSUSBD_EPINTEN_BUFFULLIEN_Msk |
                             HSUSBD_EPINTEN_SHORTRXIEN_Msk |
                             HSUSBD_EPINTEN_OUTTKIEN_Msk);
    }

    /*****************************************************/
    /*
       Generate Mass-Storage Device serial number
       To compliant USB-IF MSC test, we must enable serial string descriptor.
       However, window may fail to recognize the devices if PID/VID and serial number are all the same
       when plug them to Windows at the sample time.
       Therefore, we must generate different serial number for each device to avoid conflict
       when plug more then 2 MassStorage devices to Windows at the same time.

       NOTE: We use compiler predefine macro "__TIME__" to generate different number for serial
       at each build but each device here for a demo.
       User must change it to make sure all serial number is different between each device.
     */
    pu8 = (uint8_t *)gsHSInfo.gu8StringDesc[3];

    for (i = 0; i < 8; i++)
        pu8[pu8[0] - 16 + i * 2] = pSerial[i];
}

/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void UAC_ClassRequest(void)
{
    uint8_t buf[8];

    HSUSBD_MemCopy(buf, (void *)&gUsbCmd, 8);

    if ((buf[0] & 0x7f) == 0x22) // bmRequestType= class, endpoint
    {
        if (buf[0] & 0x80) // Device to host
        {
            switch (buf[1])
            {
                case UAC_GET_CUR:
                {
                    //uint32_t u32Temp;

                    if (buf[3] == SAMPLING_FREQ_CONTROL && (buf[4] == EPD))
                    {
                        //u32Temp = g_usbd_PlaySampleRate;
                        HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlaySampleRate, 3);
                        HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                        HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                    }
                    else if (buf[3] == SAMPLING_FREQ_CONTROL && (buf[4] == (EPC | EP_INPUT)))
                    {
                        //u32Temp = g_usbd_PlaySampleRate;
                        HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlaySampleRate, 3);
                        HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                        HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                    }
                    else
                    {
                        /* error endpoint or un identified Control*/
                        /* Setup error, stall the device */
                        HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                    }

                    break;
                }

                default:
                {
                    /* Setup error, stall the device */
                    /* unidentify CONTROL */
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                }
            }
        }
        else // Host to device
        {
            switch (buf[1])
            {
                case UAC_SET_CUR:
                {
                    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_OUTTKIEN_Msk | HSUSBD_CEPINTEN_RXPKIEN_Msk);

                    if (buf[3] == SAMPLING_FREQ_CONTROL && (buf[4] == EPD))
                    {
                        /* request to endpoint */
                        HSUSBD_CtrlOut((uint8_t *)&g_usbd_PlaySampleRate, buf[6]);

                        /* Status stage */
                        HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                        HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                        HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
                    }
                    else if (buf[3] == SAMPLING_FREQ_CONTROL && (buf[4] == (EPC | EP_INPUT)))
                    {
                        /* request to endpoint */
                        HSUSBD_CtrlOut((uint8_t *)&g_usbd_RecSampleRate, buf[6]);

                        /* Status stage */
                        HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                        HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                        HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
                    }
                    else
                    {
                        /* Setup error, stall the device */
                        /* Unidentify CONTROL*/
                        /* STALL control pipe */
                        HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                    }

                    break;
                }

                default:
                {
                    /*unimplement CONTROL or wrong endpoint number*/
                    /* Setup error, stall the device */
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                }
            }
        }
    }
    else /*Feature unit control*/
    {
        if (buf[0] & 0x80) // Device to host
        {
            switch (buf[1])
            {
                case UAC_GET_CUR:
                {
                    switch (buf[3])
                    {
                        case MUTE_CONTROL:
                        {
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMute, 1);
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMute, 1);
                            }

                            /* Data stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                            break;
                        }

                        case VOLUME_CONTROL:
                        {
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                /* Left or right channel */
                                if (buf[2] == 1)
                                {
                                    HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecVolumeL, 2);
                                }
                                else
                                {
                                    HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecVolumeR, 2);
                                }
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {
                                /* Left or right channel */
                                if (buf[2] == 1)
                                {
                                    HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayVolumeL, 2);
                                }
                                else
                                {
                                    HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayVolumeR, 2);
                                }
                            }

                            /* Data stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                            break;
                        }

                        default:
                        {
                            /* Setup error, stall the device */
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                        }
                    }

                    break;
                }

                case UAC_GET_MIN:
                {
                    switch (buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMinVolume, 2);
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMinVolume, 2);
                            }

                            /* Data stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                            break;
                        }

                        default:
                            /* STALL control pipe */
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                    }

                    break;
                }

                case UAC_GET_MAX:
                {
                    switch (buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMaxVolume, 2);
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMaxVolume, 2);
                            }

                            /* Data stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                            break;
                        }

                        default:
                            /* STALL control pipe */
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                    }

                    break;
                }

                case UAC_GET_RES:
                {
                    switch (buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecResVolume, 2);
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {
                                HSUSBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayResVolume, 2);
                            }

                            /* Data stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                            break;
                        }

                        default:
                            /* STALL control pipe */
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                    }

                    break;
                }

                default:
                {
                    /* Setup error, stall the device */
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                }
            }
        }
        else
        {
            // Host to device
            switch (buf[1])
            {
                case UAC_SET_CUR:
                {
                    switch (buf[3])
                    {
                        case MUTE_CONTROL:
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                /* request to endpoint */
                                HSUSBD_CtrlOut((uint8_t *)&g_usbd_RecMute, buf[6]);
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {
                                /* request to endpoint */
                                HSUSBD_CtrlOut((uint8_t *)&g_usbd_PlayMute, buf[6]);
                            }

                            /* Status stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
                            break;

                        case VOLUME_CONTROL:
                            if (REC_FEATURE_UNITID == buf[5])
                            {
                                if (buf[2] == 1)
                                {
                                    /* Prepare the buffer for new record volume of left channel */
                                    /* request to endpoint */
                                    HSUSBD_CtrlOut((uint8_t *)&g_usbd_RecVolumeL, buf[6]);
                                }
                                else
                                {
                                    /* Prepare the buffer for new record volume of right channel */
                                    /* request to endpoint */
                                    HSUSBD_CtrlOut((uint8_t *)&g_usbd_RecVolumeR, buf[6]);
                                }
                            }
                            else if (PLAY_FEATURE_UNITID == buf[5])
                            {

                                if (buf[2] == 1)
                                {
                                    /* Prepare the buffer for new play volume of left channel */
                                    /* request to endpoint */
                                    HSUSBD_CtrlOut((uint8_t *)&g_usbd_PlayVolumeL, buf[6]);
                                }
                                else
                                {
                                    /* Prepare the buffer for new play volume of right channel */
                                    /* request to endpoint */
                                    HSUSBD_CtrlOut((uint8_t *)&g_usbd_PlayVolumeR, buf[6]);
                                }
                            }

                            /* Status stage */
                            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
                            break;

                        default:
                            /* STALL control pipe */
                            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                            break;
                    }

                    break;
                }

                // HID
                case HID_SET_REPORT:
                {
                    if (buf[3] == 2)
                    {
                        /* Request Type = Output */
                        HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_RXPKIF_Msk);
                        HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_RXPKIEN_Msk);

                        /* Status stage */
                        HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                        HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                        HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
                    }

                    break;
                }

                case HID_SET_IDLE:
                {
                    /* Status stage */
                    HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
                    break;
                }

                case HID_SET_PROTOCOL:
                default:
                {
                    /* Setup error, stall the device */
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
                    break;
                }
            }
        }
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(void)
{
    uint8_t buf[8];
    uint8_t u32AltInterface;
    uint8_t g_usbd_UsbInterface;

    HSUSBD_MemCopy(buf, (void *)&gUsbCmd, 8);

    u32AltInterface = buf[2];
    g_usbd_UsbInterface = buf[4];

    if (g_usbd_UsbInterface == 1) // Playback
    {
        /* Audio ISO OUT interface */
        if (u32AltInterface == 2) /* start 24bit play */
        {
            PD1 = 0; /* Set the logic value of AUDIO_JKEN as Low */
            g_u8AudioSpeakerState = UAC_START_AUDIO_SPEAK;
            g_u8USBTxDataLen = USB_24bit;
        }
        else if (u32AltInterface == 1) /* start 16bit play */
        {
            PD1 = 0; /* Set the logic value of AUDIO_JKEN as Low */
            g_u8AudioSpeakerState = UAC_START_AUDIO_SPEAK;
            g_u8USBTxDataLen = USB_16bit;
        }
        else if (u32AltInterface == 0) /* stop play */
        {
            PD1 = 1; /* Set the logic value of AUDIO_JKEN as Hi */
            g_u8AudioSpeakerState = UAC_STOP_AUDIO_SPEAK;
            g_u32SpeakerByteCount = 0;
            g_u32SpeakerByteCountRec = 0;
            g_u8USBPlayEn = 0;

            HSUSBD->EP[EPD].EPRSPCTL |= HSUSBD_EPRSPCTL_FLUSH_Msk;
        }
    }
    else if (g_usbd_UsbInterface == 2) // Recorder
    {
        /* Audio ISO IN interface */
        if (u32AltInterface == 1) /* start record */
        {
            g_u8USBRecEn = 1;
            g_u8AudioRecorderState = UAC_START_AUDIO_RECORD;

            HSUSBD->EP[EPC].EPRSPCTL = HSUSBD_EPRSPCTL_ZEROLEN_Msk;
        }
        else if (u32AltInterface == 0) /* stop record */
        {
            g_u8AudioRecorderState = UAC_STOP_AUDIO_RECORD;
            g_u8USBRecEn = 0;

            HSUSBD->EP[EPC].EPRSPCTL = HSUSBD_EPRSPCTL_ZEROLEN_Msk;
            HSUSBD->EP[EPC].EPRSPCTL |= HSUSBD_EPRSPCTL_FLUSH_Msk;
        }
    }
}

/**
 * @brief       Adjust Codec PLL
 *
 * @param[in]   r   Resample state
 *
 * @return      None
 *
 * @details     This function is used to adjust NAU88C22 PLL setting to speed up or slow down I2S clock
 */
void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    static uint16_t tb0[3][3] =
    {
        {0x00C, 0x093, 0x0E9},  // 8.192
        {0x00E, 0x1D2, 0x1E3},  // * 1.005 = 8.233
        {0x009, 0x153, 0x1EF}
    }; // * .995 = 8.151
    static uint16_t tb1[3][3] =
    {
        {0x021, 0x131, 0x026},  // 7.526
        {0x024, 0x010, 0x0C5},  // * 1.005 = 7.563
        {0x01F, 0x076, 0x191}
    }; // * .995 = 7.488
    static RESAMPLE_STATE_T current = E_RS_NONE;
    int i, s;

    if (r == current)
        return;
    else
        current = r;

    switch (r)
    {
        case E_RS_UP:
            s = 1;
            break;

        case E_RS_DOWN:
            s = 2;
            break;

        case E_RS_NONE:
        default:
            s = 0;
    }

    if ((g_u32I2S0SampleRate % 8) == 0)
    {
        for (i = 0; i < 3; i++)
            I2C_WriteNAU88C22(37 + i, tb0[s][i]);
    }
    else
    {
        for (i = 0; i < 3; i++)
            I2C_WriteNAU88C22(37 + i, tb1[s][i]);
    }
}

/**
 * @brief       Adjust PLL
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to check the ring buffer status and call AdjustCodecPll() to adjust codec PLL.
 */
void Adjust_PLL(void)
{
    if (g_u8USBPlayEn)
    {
        if (g_u16PlayBack_Write_Ptr >= g_u16PlayBack_Read_Ptr)
        {
            if (g_u16PlayBack_Write_Ptr == g_u16PlayBack_Read_Ptr)
            {
                if (g_bOverFlow == TRUE)
                    g_u16PlayBack_Ptrs_Distance = g_u16PlayBack_MAX_USB_BUFFER_LEN;
                else
                    g_u16PlayBack_Ptrs_Distance = 0;
            }
            else
                g_u16PlayBack_Ptrs_Distance = g_u16PlayBack_Write_Ptr - g_u16PlayBack_Read_Ptr;
        }
        else
        {
            g_u16PlayBack_Ptrs_Distance = (g_u16PlayBack_MAX_USB_BUFFER_LEN - g_u16PlayBack_Read_Ptr) + g_u16PlayBack_Write_Ptr;
        }

        // Ring buffer is close to overflow.
        if (g_u16PlayBack_Ptrs_Distance >= g_u16PlayBack_USB_BUFF_UPPER_THRE)
        {
            AdjustCodecPll(E_RS_DOWN);
        }
        // Ring buffer is close to underflow.
        else if (g_u16PlayBack_Ptrs_Distance <= (g_u16PlayBack_USB_BUFF_LOWER_THRE / 4 * 3))
        {
            AdjustCodecPll(E_RS_UP);
        }
        else
        {
            AdjustCodecPll(E_RS_NONE);
        }
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
