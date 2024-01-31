/**************************************************************************//**
 * @file     mp3recorder.c
 * @version  V3.00
 * @brief    MP3 file audio recoder.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "l3.h"

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
static int32_t i32Cnt = 0;
shine_config_t config;
shine_t        s;
int32_t        samples_per_pass;
FIL            mp3FileObject;
size_t         ReturnSize;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
/* Write out the MP3 file */
int32_t Write_MP3(long bytes, void *buffer, void *config)
{
    f_write(&mp3FileObject, buffer, bytes, &ReturnSize);

    if (i32Cnt++ >= (bytes / 10))
    {
        i32Cnt = 0;
        printf(".");
    }

    return bytes;
}

/* Print some info about what we're going to encode */
static void check_config(shine_config_t *config)
{
    static char *version_names[4] = { "2.5", "reserved", "II", "I" };
    static char *mode_names[4]    = { "stereo", "joint-stereo", "dual-channel", "mono" };
    static char *demp_names[4]    = { "none", "50/15us", "", "CITT" };

    printf("MPEG-%s layer III, %s  Psychoacoustic Model: Shine\n",
           version_names[shine_check_config(config->wave.samplerate, config->mpeg.bitr)],
           mode_names[config->mpeg.mode]);
    printf("Bitrate: %d kbps  ", config->mpeg.bitr);
    printf("De-emphasis: %s   %s %s\n\n",
           demp_names[config->mpeg.emph],
           ((config->mpeg.original) ? "Original" : ""),
           ((config->mpeg.copyright) ? "(C)" : ""));
}

/* Use these default settings, can be overridden */
static void set_defaults(shine_config_t *config)
{
    shine_set_config_mpeg_defaults(&config->mpeg);
}

void Recorder_Init(void)
{
    FRESULT res;

    /* Open I2S0 interface and set to slave mode, audio format, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, REC_SAMPLE_RATE, I2S_DATABIT_16, REC_FORMAT, I2S_FORMAT_I2S);

    NVIC_EnableIRQ(I2S0_IRQn);
    NVIC_SetPriority(I2S0_IRQn, 4);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

    /* NAU8822 will store data in left channel */
    I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_LEFT);

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Configure NAU8822 to specific sample rate */
    NAU8822_ConfigSampleRate(REC_SAMPLE_RATE);

    /* Enable RX threshold level interrupt */
    I2S_EnableInt(I2S0, I2S_IEN_RXTHIEN_Msk);

    res = f_open(&mp3FileObject, MP3_FILE, FA_CREATE_ALWAYS | FA_WRITE);

    if (res != FR_OK)
    {
        printf("Open file error!\n");
        return;
    }

    set_defaults(&config);

    if (REC_FORMAT == I2S_MONO)
        config.wave.channels = 1;
    else
        config.wave.channels = 2;

    config.wave.samplerate = REC_SAMPLE_RATE;
    config.mpeg.bitr = REC_BIT_RATE;

    /* See if samplerate and bitrate are valid */
    if (shine_check_config(config.wave.samplerate, config.mpeg.bitr) < 0)
        printf("Unsupported samplerate/bitrate configuration.");

    /* Set to stereo mode if wave data is stereo, mono otherwise */
    if (config.wave.channels > 1)
        config.mpeg.mode = STEREO;
    else
        config.mpeg.mode = MONO;

    /* Initiate encoder */
    s = shine_initialise(&config);

    check_config(&config);

    samples_per_pass = shine_samples_per_pass(s);
}

void MP3Recorder(void)
{
    int32_t i32Written;
    uint8_t *pu8Data;
    uint32_t u32BuffPos = 0;
    uint32_t u32Len;

    u32Len = samples_per_pass * config.wave.channels;

    printf("Stop ...\n");
    printf("Encode and write out the MP3 file ");

    for (u32BuffPos = 0; u32BuffPos < g_u32BuffPos; u32BuffPos += 2 * u32Len)
    {
        pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(SPIM_HYPER_DMM0_SADDR + u32BuffPos), (int *)&i32Written);

        if (Write_MP3(i32Written, pu8Data, &config) != i32Written)
        {
            printf("shineenc: write error\n");
        }
    }

    g_u32BuffPos = 0;
}
