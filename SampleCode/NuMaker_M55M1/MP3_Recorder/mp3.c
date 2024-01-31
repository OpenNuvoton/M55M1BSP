/*
 * libmad - MPEG audio decoder library
 * Copyright (C) 2000-2004 Underbit Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: minimad.c,v 1.4 2004/01/23 09:41:32 rob Exp $
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "mad.h"

/*
 * This is perhaps the simplest example use of the MAD high-level API.
 * Standard input is mapped into memory via mmap(), then the high-level API
 * is invoked with three callbacks: input, output, and error. The output
 * callback converts MAD's high-resolution PCM samples to 16 bits, then
 * writes them to standard output in little-endian, mono-interleaved
 * format.
 */

struct mad_stream   Stream;
struct mad_frame    Frame;
struct mad_synth    Synth;

FILINFO             Finfo;
size_t              ReadSize;
size_t              Remaining;

extern FIL          mp3FileObject;
extern size_t       ReturnSize;

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
// I2S PCM buffer x2
int32_t g_ai32PCMBuffer[2][PCM_BUFFER_SIZE];
// File IO buffer for MP3 library
uint8_t g_au8MadInputBuffer[FILE_IO_BUFFER_SIZE + MAD_BUFFER_GUARD];
// buffer full flag x2
volatile uint8_t g_au8PCMBuffer_Full[2] = {0, 0};
// audio information structure
struct AudioInfoObject audioInfo;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
/* Parse MP3 header and get some informations */
void MP3_ParseHeaderInfo(uint8_t *pFileName)
{
    FRESULT res;
    uint32_t fptr = 0;

    res = f_open(&mp3FileObject, (void *)pFileName, FA_OPEN_EXISTING | FA_READ);

    if (res == FR_OK)
    {
        printf("file is opened!!\r\n");
        f_stat((void *)pFileName, &Finfo);
        audioInfo.playFileSize = Finfo.fsize;

        while (1)
        {
            res = f_read(&mp3FileObject, (int8_t *)(&g_au8MadInputBuffer[0]), FILE_IO_BUFFER_SIZE, &ReturnSize);

            /* Parsing MP3 header */
            mp3CountV1L3Headers((uint8_t *)(&g_au8MadInputBuffer[0]), ReturnSize);

            if (audioInfo.mp3SampleRate != 0)
                /* Got the header and sampling rate */
                break;

            /* ID3 may too long, try to parse following data */
            /* but only forward file point to half of buffer to prevent the header is */
            /* just right at the boundry of buffer */
            fptr += FILE_IO_BUFFER_SIZE / 2;

            if (fptr >= audioInfo.playFileSize)
                /* Fail to find header */
                break;

            f_lseek(&mp3FileObject, fptr);
        }
    }
    else
    {
        //printf("Open File Error\r\n");
        return;
    }

    f_close(&mp3FileObject);

    printf("====[MP3 Info]======\r\n");
    printf("FileSize = %d\r\n", audioInfo.playFileSize);
    printf("SampleRate = %d\r\n", audioInfo.mp3SampleRate);
    printf("BitRate = %d\r\n", audioInfo.mp3BitRate);
    printf("Channel = %d\r\n", audioInfo.mp3Channel);
    printf("=====================\r\n");
}

/* Enable I2S TX with PDMA function */
void StartPlay(void)
{
    printf("Start playing ...\n");
    PDMA_Init();
    I2S_ENABLE_TXDMA(I2S0);
    I2S_ENABLE_TX(I2S0);

    /* Enable sound output */
    audioInfo.mp3Playing = 1;
}

/* Disable I2S TX with PDMA function */
void StopPlay(void)
{
    I2S_DISABLE_TXDMA(I2S0);
    I2S_DISABLE_TX(I2S0);

    PDMA_Close(PDMA0);

    /* Disable sound output */
    audioInfo.mp3Playing = 0;
    printf("Stop ...\n\n");
}

/* MP3 decode player */
void MP3Player(void)
{
    FRESULT res;
    uint8_t *ReadStart;
    uint8_t *GuardPtr;
    volatile uint8_t u8PCMBufferTargetIdx = 0;
    volatile uint32_t pcmbuf_idx, i;
    volatile uint32_t Mp3FileOffset = 0;
    uint16_t sampleL, sampleR;

    pcmbuf_idx = 0;
    g_u8PCMBuffer_Playing = 0;
    memset((void *)&audioInfo, 0, sizeof(audioInfo));
    memset((void *)g_au8MadInputBuffer, 0, sizeof(g_au8MadInputBuffer));
    memset((void *)g_ai32PCMBuffer, 0, sizeof(g_ai32PCMBuffer));
    memset((void *)g_au8PCMBuffer_Full, 0, sizeof(g_au8PCMBuffer_Full));

    /* Parse MP3 header */
    MP3_ParseHeaderInfo((uint8_t *)MP3_FILE);

    /* First the structures used by libmad must be initialized */
    mad_stream_init(&Stream);
    mad_frame_init(&Frame);
    mad_synth_init(&Synth);

    /* Open MP3 file */
    res = f_open(&mp3FileObject, MP3_FILE, FA_OPEN_EXISTING | FA_READ);

    if (res != FR_OK)
    {
        //printf("Open file error \r\n");
        return;
    }

    /* Open I2S0 interface and set to slave mode, audio format, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, PLAY_FORMAT, I2S_FORMAT_I2S);
    NVIC_EnableIRQ(I2S0_IRQn);

    /* Set JK-EN low to enable phone jack on NuMaker board. */
    SET_GPIO_PB12();
    GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);
    PB12 = 0;

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    I2S0->CTL1 |= I2S_CTL1_PBWIDTH_Msk;

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Configure NAU8822 to specific sample rate */
    NAU8822_ConfigSampleRate(audioInfo.mp3SampleRate);

    while (1)
    {
        if (Stream.buffer == NULL || Stream.error == MAD_ERROR_BUFLEN)
        {
            if (Stream.next_frame != NULL)
            {
                /* Get the remaining frame */
                Remaining = Stream.bufend - Stream.next_frame;
                memmove(g_au8MadInputBuffer, Stream.next_frame, Remaining);
                ReadStart = g_au8MadInputBuffer + Remaining;
                ReadSize = FILE_IO_BUFFER_SIZE - Remaining;
            }
            else
            {
                ReadSize = FILE_IO_BUFFER_SIZE,
                ReadStart = g_au8MadInputBuffer,
                Remaining = 0;
            }

            /* Read the file */
            res = f_read(&mp3FileObject, ReadStart, ReadSize, &ReturnSize);

            if (res != FR_OK)
            {
                printf("Stop !(%x)\n\r", res);
                goto stop;
            }

            if (f_eof(&mp3FileObject))
            {
                if (ReturnSize == 0)
                    goto stop;
            }

            /* If the file is over */
            if (ReadSize > ReturnSize)
            {
                GuardPtr = ReadStart + ReturnSize;
                memset(GuardPtr, 0, MAD_BUFFER_GUARD);
                ReturnSize += MAD_BUFFER_GUARD;
            }

            Mp3FileOffset = Mp3FileOffset + ReturnSize;
            /* Pipe the new buffer content to libmad's stream decoder facility */
            mad_stream_buffer(&Stream, g_au8MadInputBuffer, ReturnSize + Remaining);
            Stream.error = (enum  mad_error)0;
        }

        /* Decode a frame from the mp3 stream data */
        if (mad_frame_decode(&Frame, &Stream))
        {
            if (MAD_RECOVERABLE(Stream.error))
            {
                /*
                if(Stream.error!=MAD_ERROR_LOSTSYNC || Stream.this_frame!=GuardPtr)
                {
                }
                */
                continue;
            }
            else
            {
                /* The current frame is not full, need to read the remaining part */
                if (Stream.error == MAD_ERROR_BUFLEN)
                {
                    continue;
                }
                else
                {
                    printf("Something error!!\n");

                    /* Play the next file */
                    audioInfo.mp3FileEndFlag = 1;
                    goto stop;
                }
            }
        }

        /* Once decoded the frame is synthesized to PCM samples. No errors
           are reported by mad_synth_frame();
        */
        mad_synth_frame(&Synth, &Frame);

        /* decode finished, try to copy pcm data to audio buffer */

        if (audioInfo.mp3Playing)
        {
            /* If next buffer is still full (playing), wait until it's empty */
            if (g_au8PCMBuffer_Full[u8PCMBufferTargetIdx] == 1)
                while (g_au8PCMBuffer_Full[u8PCMBufferTargetIdx]);
        }
        else
        {

            if ((g_au8PCMBuffer_Full[0] == 1) && (g_au8PCMBuffer_Full[1] == 1))         /* All buffers are full, wait */
            {
                StartPlay();
            }
        }

        for (i = 0; i < (int32_t)Synth.pcm.length; i++)
        {
            /* Get the left/right samples */
            sampleL = Synth.pcm.samples[0][i];
            sampleR = Synth.pcm.samples[1][i];

            /* Fill PCM data to I2S(PDMA) buffer */
            g_ai32PCMBuffer[u8PCMBufferTargetIdx][pcmbuf_idx++] = sampleL;

            /* Need change buffer ? */
            if (pcmbuf_idx == PCM_BUFFER_SIZE)
            {
                g_au8PCMBuffer_Full[u8PCMBufferTargetIdx] = 1;      /* Set full flag */
                u8PCMBufferTargetIdx ^= 1;

                pcmbuf_idx = 0;

                //printf("change to ==>%d ..\n", u8PCMBufferTargetIdx);
                /* If next buffer is still full (playing), wait until it's empty */
                if ((g_au8PCMBuffer_Full[u8PCMBufferTargetIdx] == 1) && (audioInfo.mp3Playing))
                    while (g_au8PCMBuffer_Full[u8PCMBufferTargetIdx]);
            }
        }
    }

stop:

    printf("Exit MP3\r\n");

    mad_synth_finish(&Synth);
    mad_frame_finish(&Frame);
    mad_stream_finish(&Stream);

    f_close(&mp3FileObject);
    StopPlay();
}
