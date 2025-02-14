/**************************************************************************//**
 * @file     wavplayer.c
 * @version  V1.00
 * @brief    Wave file audio player.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"

/*
 * This is perhaps the simplest example use of the MAD high-level API.
 * Standard input is mapped into memory via mmap(), then the high-level API
 * is invoked with three callbacks: input, output, and error. The output
 * callback converts MAD's high-resolution PCM samples to 16 bits, then
 * writes them to standard output in little-endian, stereo-interleaved
 * format.
 */

static FIL    wavFileObject;
static size_t ReturnSize;

#if (NVT_DCACHE_ON == 1)
    // If DCACHE is enabled, use a cache-line aligned buffer for the I2S PCM DMA
    signed int aiPCMBuffer[DCACHE_ALIGN_LINE_SIZE(2)][DCACHE_ALIGN_LINE_SIZE(PCM_BUFFER_SIZE)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // If DCACHE is not enabled, use a standard buffer for the I2S PCM DMA
    signed int aiPCMBuffer[2][PCM_BUFFER_SIZE];
#endif
volatile uint8_t g_u8PCMBufferFull[2] = {0, 0};
volatile uint8_t g_u8PCMBufferPlaying = 0;
static uint32_t s_au32WavHeader[11];

void WAVPlayer(void)
{
    FRESULT res;
    uint8_t u8PCMBufferTargetIdx = 0;
    uint32_t u32WavSamplingRate;

    res = f_open(&wavFileObject, "0:\\test.wav", FA_OPEN_EXISTING | FA_READ);       //USBH:0 , SD0: 1

    if (res != FR_OK)
    {
        printf("Open file error!\n");
        return;
    }

    // read sampling rate from WAV header
    memset(s_au32WavHeader, 0, sizeof(s_au32WavHeader));
    f_read(&wavFileObject, s_au32WavHeader, 44, &ReturnSize);
    u32WavSamplingRate = s_au32WavHeader[6];

#if NAU8822
    /* Configure NAU8822 to specific sample rate */
    NAU8822_ConfigSampleRate(u32WavSamplingRate);
#else
    /* Configure NAU88L25 to specific sample rate */
    NAU88L25_ConfigSampleRate(u32WavSamplingRate);
#endif
    printf("wav: sampling rate=%d\n", u32WavSamplingRate);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;

    while (1)
    {
#if (NVT_DCACHE_ON == 1)
        // If DCACHE is enabled, make sure the I2S PCM DMA buffer is cleaned and invalidated
        // This is to ensure that the data written to the cache is actually written to the memory
        SCB_CleanInvalidateDCache_by_Addr((int *)&aiPCMBuffer, sizeof(aiPCMBuffer));
#endif

        if ((g_u8PCMBufferFull[0] == 1) && (g_u8PCMBufferFull[1] == 1))         //all buffers are full, wait
        {
            if (!u8AudioPlaying)
            {
                u8AudioPlaying = 1;
                I2S_ENABLE_TXDMA(I2S0);
                I2S_ENABLE_TX(I2S0);
                printf("Start Playing ...\n");
            }

            while ((g_u8PCMBufferFull[0] == 1) && (g_u8PCMBufferFull[1] == 1));

            //printf(".");
        }

        res = f_read(&wavFileObject, &aiPCMBuffer[u8PCMBufferTargetIdx][0], PCM_BUFFER_SIZE * 4, &ReturnSize);

        if (ReturnSize < PCM_BUFFER_SIZE * 4)
            memset(&aiPCMBuffer[u8PCMBufferTargetIdx][ReturnSize], 0, PCM_BUFFER_SIZE * 4 - ReturnSize);

        if (f_eof(&wavFileObject) && (ReturnSize == 0))
            break;

        g_u8PCMBufferFull[u8PCMBufferTargetIdx] = 1;

        if (u8AudioPlaying)
        {
            if (g_u8PCMBufferFull[u8PCMBufferTargetIdx ^ 1] == 1)
                while (g_u8PCMBufferFull[u8PCMBufferTargetIdx ^ 1]);
        }

        u8PCMBufferTargetIdx ^= 1;

        //printf("change to ==>%d\n", u8PCMBufferTargetIdx);
    }

    printf("Done..\n");
    I2S_DISABLE_TX(I2S0);
    I2S_DISABLE_TXDMA(I2S0);
    f_close(&wavFileObject);

    u8AudioPlaying = 0;
    g_u8PCMBufferFull[0] = 0;
    g_u8PCMBufferFull[1] = 0;
}
