/**************************************************************************//**
 * @file     DMICRecord.h
 * @version  V1.00
 * @brief    DMIC audio record function
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DMIC_RECORD_H__
#define __DMIC_RECORD_H__

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

// Init DMIC record resource
int32_t DMICRecord_Init(
    uint32_t u32SampleRate,
    uint32_t u32Channels,
    uint32_t u32BlockSamples,
    uint32_t u32BlockCounts
);

// DMIC start record
int32_t DMICRecord_StartRec(void);

// DMIC stop record
int32_t DMICRecord_StopRec(void);

// Get available audio sample number
int32_t DMICRecord_AvailSamples(void);

// Read audio sample data
int32_t DMICRecord_ReadSamples(int16_t *pi16SampleData, uint32_t u32Samples);

// Update audio sample data index
int32_t DMICRecord_UpdateReadSampleIndex(uint32_t u32Samples);

// Un-init DMIC record
void DMICRecord_UnInit(void);

#ifdef __cplusplus
}
#endif

#endif

