/********************************************************************************
 *                                                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.                              *
 *                                                                                              *
 ********************************************************************************/

#include "NuMicro.h"
#include "WavFileUtil.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ff.h"

static FIL     wavFileObject;
static size_t  ReturnSize;
static FILINFO finfo;
static PCSTR   s_pszOutFileName;

//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_WriteHeader(
    S_WavFileWriteInfo *psInfo
)
{
    const BYTE abySignature1[4] = { 'R', 'I', 'F', 'F' };

    f_write(&wavFileObject, abySignature1, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    psInfo->u32RiffDataLenFilePos = 4;
    UINT32 u32Len = 0;

    f_write(&wavFileObject, &u32Len, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    const BYTE abySignature2[4] = { 'W', 'A', 'V', 'E' };

    f_write(&wavFileObject, abySignature2, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    return TRUE;
} // WavFileUtil_Write_WriteHeader()


//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_SetFormatInternal(
    S_WavFileWriteInfo *psInfo,
    void               *pWavFormat,
    UINT32              u32WavFormatDataLen
)
{
    if (!psInfo || psInfo->i32FileHandle < 0)
        return FALSE;

    // "fmt " chunk
    static const BYTE abySignature1[4] = { 'f', 'm', 't', ' ' };

    f_write(&wavFileObject, abySignature1, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    f_write(&wavFileObject, &u32WavFormatDataLen, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    f_write(&wavFileObject, pWavFormat, u32WavFormatDataLen, &ReturnSize);

    if (u32WavFormatDataLen != ReturnSize)
        return FALSE;

    // "data" chunk
    static const BYTE abySignature2[4] = { 'd', 'a', 't', 'a' };

    f_write(&wavFileObject, abySignature2, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    psInfo->u32DataChunkLenFilePos = 42;
    UINT32 u32ChunkLen = 0;

    f_write(&wavFileObject, &u32ChunkLen, 4, &ReturnSize);

    if (4 != ReturnSize)
        return FALSE;

    return TRUE;
} // WavFileUtil_Write_SetFormatInternal()


//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_SetFormatEx(
    S_WavFileWriteInfo *psInfo,
    S_WavFormatEx      *psWavFormatEx
)
{
    return WavFileUtil_Write_SetFormatInternal(
               psInfo, psWavFormatEx, WAV_FORMAT_EX_LEN);
} // WavFileUtil_Write_SetFormatEx()


//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_SetIMAFormat(
    S_WavFileWriteInfo *psInfo,
    S_IMAWavFormat     *psIMAWavFormat
)
{
    return WavFileUtil_Write_SetFormatInternal(
               psInfo, psIMAWavFormat, IMA_WAV_FORMAT_LEN);
} // WavFileUtil_Write_SetIMAFormat()


//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_FillFormatEx(
    S_WavFormatEx  *psWavFormatEx,
    UINT16          u16Channels,
    UINT32          u32SamplingRate,
    UINT16          u16BitsPerSample    // 8 or 16
)
{
    if ((8 != u16BitsPerSample) && (16 != u16BitsPerSample))
        return FALSE;

    psWavFormatEx->u16FormatTag = 0x0001;                   // PCM
    psWavFormatEx->u16Channels = u16Channels;
    psWavFormatEx->u32SamplesPerSec = u32SamplingRate;
    psWavFormatEx->u16BlockAlign = u16BitsPerSample / 8;    // (N + 1) * 4 * nChannels
    psWavFormatEx->u16BitsPerSample = u16BitsPerSample;
    psWavFormatEx->u16CbSize = 0;
    psWavFormatEx->u32AvgBytesPerSec =
        (u32SamplingRate * psWavFormatEx->u16BlockAlign);
    return TRUE;
} // WavFileUtil_Write_FillFormatEx()


//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_FillFormatG726(
    S_WavFormatEx  *psWavFormatEx,
    UINT16          u16Channels,
    UINT32          u32SamplingRate,
    UINT16          u16BitsPerSample    // 8 or 16
)
{
    //  if ((2 != u16BitsPerSample) && (3 != u16BitsPerSample))
    //      return FALSE;

    psWavFormatEx->u16FormatTag = eWAVE_FORMAT_G726_ADPCM;  // G726
    psWavFormatEx->u16Channels = u16Channels;
    psWavFormatEx->u32SamplesPerSec = u32SamplingRate;
    psWavFormatEx->u16BlockAlign = u16Channels * u16BitsPerSample / 8;
    psWavFormatEx->u16BitsPerSample = u16BitsPerSample;
    psWavFormatEx->u16CbSize = 0;
    psWavFormatEx->u32AvgBytesPerSec =
        (u32SamplingRate * psWavFormatEx->u16BlockAlign);
    return TRUE;
} // WavFileUtil_Write_FillFormatG726()


//----------------------------------------------------------------------------
static BOOL WavFileUtil_Write_FillIMAFormat(
    S_IMAWavFormat *psIMAWavFormat,
    UINT16          u16Channels,
    UINT32          u32SamplingRate,
    UINT16          u16BitsPerSample
)
{
    if ((3 != u16BitsPerSample) && (4 != u16BitsPerSample))
        return FALSE;

    UINT16 u16N;

    if (u32SamplingRate < 22050)
        u16N = 63;
    else if (u32SamplingRate < 44100)
    {
        if (3 == u16BitsPerSample)
            u16N = 126;
        else
            u16N = 127;
    }
    else
        u16N = 255;

    psIMAWavFormat->u16FormatTag = 0x0011;                          // DVI ADPCM
    psIMAWavFormat->u16Channels = u16Channels;
    psIMAWavFormat->u32SamplesPerSec = u32SamplingRate;
    psIMAWavFormat->u16BlockAlign = (u16N + 1) * 4 * u16Channels;   // (N + 1) * 4 * nChannels
    psIMAWavFormat->u16BitsPerSample = u16BitsPerSample;
    psIMAWavFormat->u16CbSize = 2;

    if (3 == u16BitsPerSample)
        psIMAWavFormat->u16SamplesPerBlock = (u16N * 32) / 3 + 1;   // ((N * 4 * 8) / 3) + 1
    else
        psIMAWavFormat->u16SamplesPerBlock = (u16N * 8) + 1;        // ((N * 4 * 8) / 4) + 1

    psIMAWavFormat->u32AvgBytesPerSec =
        (u32SamplingRate * psIMAWavFormat->u16BlockAlign) /
        psIMAWavFormat->u16SamplesPerBlock;
    return TRUE;
} // WavFileUtil_Write_FillIMAFormat()


//----------------------------------------------------------------------------
// Public functions
//----------------------------------------------------------------------------
BOOL WavFileUtil_Write_Initialize(
    S_WavFileWriteInfo *psInfo,
    PCSTR               pszOutFileName

)
{
    FRESULT res;

    if (NULL == psInfo)
        return FALSE;

    res = f_open(&wavFileObject, (const TCHAR *)pszOutFileName, FA_CREATE_ALWAYS | FA_WRITE);      //USBH:0 , SD0: 1

    if (res != FR_OK)
    {
        printf("Open file %s error!\n", pszOutFileName);
        return FALSE;
    }

    psInfo->i32FileHandle = (INT32)&wavFileObject;

    if (psInfo->i32FileHandle < 0)
        return FALSE;

    psInfo->u32DataChunkLenFilePos = 0;
    psInfo->u32RiffDataLenFilePos = 0;

    if (!WavFileUtil_Write_WriteHeader(psInfo))
    {
        f_close(&wavFileObject);
        psInfo->i32FileHandle = -1;
        return FALSE;
    }

    s_pszOutFileName = pszOutFileName;
    return TRUE;
} // WavFileUtil_Write_Initialize()


//----------------------------------------------------------------------------
BOOL WavFileUtil_Write_SetFormat(
    S_WavFileWriteInfo *psInfo,
    E_WavFormatTag      eWavFormatTag,
    UINT16              u16Channels,
    UINT32              u32SamplingRate,
    UINT16              u16BitsPerSample
)
{
    S_WavFormatEx   sWavFormatEx;

    switch (eWavFormatTag)
    {
        case eWAVE_FORMAT_PCM:
            if (!WavFileUtil_Write_FillFormatEx(&sWavFormatEx, u16Channels,
                                                u32SamplingRate, u16BitsPerSample))
                return FALSE;

            if (!WavFileUtil_Write_SetFormatEx(psInfo, &sWavFormatEx))
                return FALSE;

            break;

        case eWAVE_FORMAT_IMA_ADPCM:
        {
            S_IMAWavFormat  sIMAWavFormat;

            if (!WavFileUtil_Write_FillIMAFormat(&sIMAWavFormat, u16Channels,
                                                 u32SamplingRate, u16BitsPerSample))
                return FALSE;

            if (!WavFileUtil_Write_SetIMAFormat(psInfo, &sIMAWavFormat))
                return FALSE;

            break;
        }

        case eWAVE_FORMAT_G726_ADPCM:
            if (!WavFileUtil_Write_FillFormatG726(&sWavFormatEx, u16Channels,
                                                  u32SamplingRate, u16BitsPerSample))
                return FALSE;

            if (!WavFileUtil_Write_SetFormatEx(psInfo, &sWavFormatEx))
                return FALSE;

            break;

        default:
            return FALSE;
    } // switch eWavFormatTag

    return TRUE;
} // WavFileUtil_Write_SetFormat()


//----------------------------------------------------------------------------
BOOL WavFileUtil_Write_WriteData(
    S_WavFileWriteInfo *psInfo,
    const BYTE         *pbyData,
    UINT32              u32DataSize
)
{
    if (!psInfo || psInfo->i32FileHandle < 0)
        return FALSE;

    f_write(&wavFileObject, pbyData, u32DataSize, &ReturnSize);

    if (u32DataSize != ReturnSize)
        return FALSE;

    return TRUE;
} // WavFileUtil_Write_WriteData()


//----------------------------------------------------------------------------
BOOL WavFileUtil_Write_Finish(
    S_WavFileWriteInfo *psInfo
)
{
    if (!psInfo || psInfo->i32FileHandle < 0)
        return FALSE;

    if (0 != f_close(&wavFileObject))
        return FALSE;

    f_stat((const TCHAR *) s_pszOutFileName, &finfo);

    UINT32 u32Pos = finfo.fsize;

    FRESULT res;

    res = f_open(&wavFileObject, (const TCHAR *) s_pszOutFileName, FA_OPEN_EXISTING | FA_WRITE);      //USBH:0 , SD0: 1

    if (res != FR_OK)
    {
        printf("!!!Open file error!\n");
        return FALSE;
    }

    if ((u32Pos > psInfo->u32RiffDataLenFilePos) &&
            (u32Pos > psInfo->u32DataChunkLenFilePos))
    {
        UINT32  u32Len;

        u32Len = u32Pos - psInfo->u32RiffDataLenFilePos - 4;

        if (0 != f_lseek(&wavFileObject, psInfo->u32RiffDataLenFilePos))
            return FALSE;

        f_write(&wavFileObject, &u32Len, 4, &ReturnSize);

        if (4 != ReturnSize)
            return FALSE;

        u32Len = u32Pos - psInfo->u32DataChunkLenFilePos - 4;

        if (0 != f_lseek(&wavFileObject, psInfo->u32DataChunkLenFilePos))
            return FALSE;

        f_write(&wavFileObject, &u32Len, 4, &ReturnSize);

        if (4 != ReturnSize)
            return FALSE;
    }

    if (0 != f_close(&wavFileObject))
        return FALSE;

    psInfo->i32FileHandle = -1;
    return TRUE;
} // WavFileUtil_Write_Finish()

DWORD get_fattime(void)
{
    DWORD g_u64Tmr;

    g_u64Tmr = 0x00000;

    return g_u64Tmr;
}
