/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * @brief    I2S wave player sample configuration header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

#include "ff.h"

#define NAU8822     1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN    32
#define BUFF_HALF_LEN   (BUFF_LEN/2)

#define PCM_BUFFER_SIZE 4*1024

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t endsrc;
    uint32_t enddest;
    uint32_t offset;
} DMA_DESC_T;

extern FATFS FatFs[];      /* File system object for logical drive */
extern uint8_t u8AudioPlaying;
extern signed int aiPCMBuffer[2][PCM_BUFFER_SIZE];
extern volatile uint8_t g_u8PCMBufferFull[2];
extern volatile uint8_t g_u8PCMBufferPlaying;

void WAVPlayer(void);
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate);
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate);

#endif
