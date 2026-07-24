/******************************************************************************
 * @file     uac_loopback.c
 * @version  V1.00
 * @brief    USBD audio sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include <stdio.h>

#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_uac.h"

#define PCM_BUF_LEN            (192*24)     /* suggest 1K at least */

/* Global variables  */
static volatile uint8_t s_u8RecEn = 0;
static volatile uint8_t s_u8PlayEn = 0;

/* Subslot byte size for mic/speaker (set by main after enumeration) */
volatile uint8_t g_u8MicSubslotSize = 2;    /* default 2 bytes (16-bit) */
volatile uint8_t g_u8SpkSubslotSize = 2;    /* default 2 bytes (16-bit) */
volatile uint8_t g_u8MicChannels = 2;       /* default stereo */
volatile uint8_t g_u8SpkChannels = 2;       /* default stereo */
volatile uint16_t g_u16SampleClockFreq = 6;       /* default 48Khz*/

volatile int8_t g_i8MicIsMono = 0;

/* UAC audio in/out PCM buffer.  */
#ifdef __ICCARM__
    #pragma data_alignment=32
    uint8_t s_au8PcmBuf[PCM_BUF_LEN];
#else
    static uint8_t s_au8PcmBuf[PCM_BUF_LEN] __attribute__((aligned(4)));
#endif
static volatile uint32_t s_u32UacRecPos = 0;       /* UAC record pointer of PCM buffer       */
static volatile uint32_t s_u32UacPlayPos = 0;      /* UAC playback pointer of PCM buffer     */
volatile uint32_t g_u32UacRecCnt = 0;       /* Counter of UAC record data             */
volatile uint32_t g_u32UacPlayCnt = 0;      /* Counter UAC playback data              */

void ResetAudioLoopBack(void);
int audio_1_0_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
int audio_1_0_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
int audio_2_0_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
int audio_2_0_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);

void ResetAudioLoopBack(void)
{
    memset(s_au8PcmBuf, 0, sizeof(s_au8PcmBuf));
    s_u32UacRecPos = 0;
    s_u32UacPlayPos = 0;
    g_u32UacRecCnt = 0;
    g_u32UacPlayCnt = 0;
    s_u8RecEn = 0;
    s_u8PlayEn = 0;
}

/**
 *  @brief  USB UAC1.0 audio-in data callback function.
 *          UAC driver deleivers an audio in data packet received from UAC device.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Audio in packet buffer
 *  @param[in] i8Len    Length of audio in packet
 *  @return   UAC driver does not check this return value.
 */
int audio_1_0_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    int i8Cnt, i8CpLen;
    uint16_t *pu16Dptr, *pu16Bptr;

    (void)dev;

    if (g_i8MicIsMono)
    {
        if (s_u32UacRecPos + (uint32_t)i8Len * 2 >= PCM_BUF_LEN)
        {
            i8CpLen = (PCM_BUF_LEN - s_u32UacRecPos) / 2;
        }
        else
        {
            i8CpLen = i8Len;
        }

        pu16Dptr = (uint16_t *)(uint32_t)pu8Data;
        pu16Bptr = (uint16_t *)(uint32_t)&s_au8PcmBuf[s_u32UacRecPos];

        for (i8Cnt = 0; i8Cnt < i8CpLen; i8Cnt += 2)
        {
            *pu16Bptr++ = *pu16Dptr;                /* 16-bit PCM data                            */
            *pu16Bptr++ = *pu16Dptr++;              /* duplicate PCM data                         */
        }

        s_u32UacRecPos = (s_u32UacRecPos + (uint32_t)i8CpLen * 2) % PCM_BUF_LEN;
        g_u32UacRecCnt += (uint32_t)i8CpLen;
        i8Len -= i8CpLen;

        if (i8Len)
        {
            pu16Dptr = (uint16_t *)(uint32_t)&pu8Data[i8CpLen];
            pu16Bptr = (uint16_t *)s_au8PcmBuf;

            for (i8Cnt = 0; i8Cnt < i8Len; i8Cnt += 2)
            {
                *pu16Bptr++ = *pu16Dptr;            /* 16-bit PCM data                            */
                *pu16Bptr++ = *pu16Dptr++;          /* duplicate PCM data                         */
            }

            s_u32UacRecPos = (uint32_t)i8Len * 2;
            g_u32UacRecCnt += (uint32_t)i8Len;
        }
    }
    else
    {
        if (s_u32UacRecPos + (uint32_t)i8Len >= PCM_BUF_LEN)
        {
            i8CpLen = PCM_BUF_LEN - (int)s_u32UacRecPos;
        }
        else
        {
            i8CpLen = i8Len;
        }

        memcpy(&s_au8PcmBuf[s_u32UacRecPos], pu8Data, (uint32_t)i8CpLen);

        s_u32UacRecPos = (s_u32UacRecPos + (uint32_t)i8CpLen) % PCM_BUF_LEN;
        g_u32UacRecCnt += (uint32_t)i8CpLen;
        i8Len -= i8CpLen;

        if (i8Len)
        {
            memcpy(&s_au8PcmBuf[0], &pu8Data[i8CpLen], (uint32_t)i8Len);
            s_u32UacRecPos = (uint32_t)i8Len;
            g_u32UacRecCnt += (uint32_t)i8Len;
        }
    }

    if ((s_u8PlayEn == 0) && (s_u32UacRecPos >= PCM_BUF_LEN / 2))
    {
        s_u32UacPlayPos = g_u32UacPlayCnt = 0;
        s_u8PlayEn = 1;
    }

    return 0;
}

/**
 *  @brief  USB UAC1.0 audio-out data callback function.
 *          UAC driver requests user to move audio-out data into the specified address. The audio-out
 *          data will then be send to UAC device via isochronous-out pipe.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Application should move audio-out data into this buffer.
 *  @param[in] i8Len    Maximum length of audio-out data can be moved.
 *  @return   Actual length of audio data moved.
 */
int audio_1_0_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    int i8CpLen;

    (void)dev;
    (void)i8Len;

    if (!s_u8PlayEn)
    {
        memset(pu8Data, 0, 192);
        g_u32UacPlayCnt += 192;
        return 192;
    }

    if (PCM_BUF_LEN - s_u32UacPlayPos < 192)
    {
        i8CpLen = PCM_BUF_LEN - (int)s_u32UacPlayPos;
    }
    else
    {
        i8CpLen = 192;
    }

    memcpy(pu8Data, &s_au8PcmBuf[s_u32UacPlayPos], (uint32_t)i8CpLen);
    s_u32UacPlayPos = (s_u32UacPlayPos + (uint32_t)i8CpLen) % PCM_BUF_LEN;

    if (i8CpLen < 192)
    {
        memcpy(&pu8Data[i8CpLen], &s_au8PcmBuf[0], 192 - (uint32_t)i8CpLen);
        s_u32UacPlayPos = 192 - (uint32_t)i8CpLen;
    }

    g_u32UacPlayCnt += 192;

    return 192;   // for 48000 stero Hz
}

/**
 *  @brief  USB UAC2.0 audio-in data callback function.
 *          UAC driver deleivers an audio in data packet received from UAC device.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Audio in packet buffer
 *  @param[in] i8Len    Length of audio in packet
 *  @return   UAC driver does not check this return value.
 */
int audio_2_0_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    int i8Idx, i8OutBytes;
    uint8_t u8MicSS = dev->asif_in.ft2->bSubslotSize;    /* mic bytes per sample  */
    uint8_t u8SpkSS = dev->asif_out.ft2->bSubslotSize;   /* spk bytes per sample  */
    uint8_t u8MicCh = g_u8MicChannels;
    uint8_t u8SpkCh = g_u8SpkChannels;
    uint8_t u8MicFrameSize, u8SpkFrameSize;
    int i8Frames;

    if ((u8MicSS == 0) || (u8MicCh == 0))
        return 0;

    u8MicFrameSize = u8MicSS * u8MicCh;     /* bytes per mic frame (all channels)        */
    u8SpkFrameSize = u8SpkSS * u8SpkCh;     /* bytes per speaker frame (all channels)    */
    i8Frames = i8Len / (int)u8MicFrameSize;  /* number of audio frames received           */

    for (i8Idx = 0; i8Idx < i8Frames; i8Idx++)
    {
        uint8_t *pu8Src = &pu8Data[i8Idx * (int)u8MicFrameSize];
        uint8_t *pu8Dst = &s_au8PcmBuf[s_u32UacRecPos];
        int i8Ch;

        /* Convert each channel sample from mic format to speaker format */
        for (i8Ch = 0; i8Ch < (int)u8SpkCh; i8Ch++)
        {
            uint8_t *pu8SrcCh;

            if (i8Ch < (int)u8MicCh)
                pu8SrcCh = &pu8Src[i8Ch * (int)u8MicSS];
            else
                pu8SrcCh = &pu8Src[0];  /* duplicate ch0 if mic has fewer channels */

            if (u8MicSS == u8SpkSS)
            {
                /* Same bit depth to direct copy */
                memcpy(&pu8Dst[i8Ch * (int)u8SpkSS], pu8SrcCh, u8SpkSS);
            }
            else if (u8MicSS > u8SpkSS)
            {
                /* Mic has more bits (e.g. 24-bit to 16-bit): take MSBs (little-endian) */
                int i8Skip = (int)u8MicSS - (int)u8SpkSS;
                memcpy(&pu8Dst[i8Ch * (int)u8SpkSS], &pu8SrcCh[i8Skip], u8SpkSS);
            }
            else
            {
                /* Mic has fewer bits (e.g. 16-bit to 24-bit): zero-pad LSBs */
                int i8Pad = (int)u8SpkSS - (int)u8MicSS;
                memset(&pu8Dst[i8Ch * (int)u8SpkSS], 0, (uint32_t)i8Pad);
                memcpy(&pu8Dst[i8Ch * (int)u8SpkSS + i8Pad], pu8SrcCh, u8MicSS);
            }
        }

        s_u32UacRecPos += (uint32_t)u8SpkFrameSize;

        if (s_u32UacRecPos >= PCM_BUF_LEN)
            s_u32UacRecPos = 0;
    }

    i8OutBytes = i8Frames * (int)u8SpkFrameSize;
    g_u32UacRecCnt += (uint32_t)i8OutBytes;

    if ((s_u8PlayEn == 0) && (s_u32UacRecPos >= PCM_BUF_LEN / 2))
    {
        s_u32UacPlayPos = g_u32UacPlayCnt = 0;
        s_u8PlayEn = 1;
    }

    return 0;
}

/**
 *  @brief  USB UAC2.0 audio-out data callback function.
 *          UAC driver requests user to move audio-out data into the specified address. The audio-out
 *          data will then be send to UAC device via isochronous-out pipe.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Application should move audio-out data into this buffer.
 *  @param[in] i8Len    Maximum length of audio-out data can be moved.
 *  @return   Actual length of audio data moved.
 */
int audio_2_0_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    int i8CpLen;
    uint8_t SpkSubslotSize = 0;

    SpkSubslotSize = dev->asif_out.ft2->bSubslotSize;
    /* Calculate actual packet size: 6 samples/us × channels × bytes_per_sample         */
    /* i8Len is wMaxPacketSize (could be much larger), we must return only 1ms of data    */
    int i8PktSize = (int)g_u16SampleClockFreq * (int)g_u8SpkChannels * (int)SpkSubslotSize;

    if (i8PktSize <= 0)
        i8PktSize = 192;   /* fallback: 48kHz stereo 16-bit */

    if (i8PktSize > i8Len)
        i8PktSize = i8Len;

    if (!s_u8PlayEn)
    {
        memset(pu8Data, 0, (uint32_t)i8PktSize);
        g_u32UacPlayCnt += (uint32_t)i8PktSize;
        return i8PktSize;
    }

    if (PCM_BUF_LEN - (int)s_u32UacPlayPos < i8PktSize)
    {
        i8CpLen = PCM_BUF_LEN - (int)s_u32UacPlayPos;
    }
    else
    {
        i8CpLen = i8PktSize;
    }

    memcpy(pu8Data, &s_au8PcmBuf[s_u32UacPlayPos], (uint32_t)i8CpLen);
    s_u32UacPlayPos = (s_u32UacPlayPos + (uint32_t)i8CpLen) % PCM_BUF_LEN;

    if (i8CpLen < i8PktSize)
    {
        memcpy(&pu8Data[i8CpLen], &s_au8PcmBuf[0], (uint32_t)(i8PktSize - i8CpLen));
        s_u32UacPlayPos = (uint32_t)(i8PktSize - i8CpLen);
    }

    g_u32UacPlayCnt += (uint32_t)i8PktSize;

    return i8PktSize;
}
