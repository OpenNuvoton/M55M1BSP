/**************************************************************************//**
 * @file     DMICRecord.c
 * @version  V1.00
 * @brief    DMIC reocrd.c
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "DMICRecord.h"
#include "usbd_audio.h"

#define DMIC_LPPDMA_CH       (2)
#define LPPDMA_BUF_ALIGH     (32)

typedef struct
{
    int32_t i32ReadSampleIndex;
    int32_t i32WriteSampleIndex;
    int32_t i32TotalSamples;
    uint32_t u32Channels;
    int16_t *pi16AudioInBuf;
    uint32_t u32PDMABlockSamples;
} S_BUF_CTRL;

// Provide LPPDMA description for ping-pong.
// The DMA descriptor table for I2S must be aligned to 32 bytes and be placed in DCache when DCache is enabled.
#if (NVT_DCACHE_ON == 1)
    // Placing the DMA descriptor table in DCache to reduce the penalty of cache miss.
    NVT_DTCM __ALIGNED(32) static LPDSCT_T s_sLPPDMA_DMIC_SCT[2];
#else
    // Placing the DMA descriptor table in normal SRAM.
    static LPDSCT_T s_sLPPDMA_DMIC_SCT[2];
#endif


static int16_t *s_pi16PDMAPingPoneBuf[2];
static uint32_t s_u32PDMAPingPoneBuf_Aligned[2];
static S_BUF_CTRL s_sAudioBufCtrl;

/* DMA scatter-gather descriptor */
#if (NVT_DCACHE_ON == 1)
    /* Descriptors are placed in a non-cacheable region */
    NVT_NONCACHEABLE __attribute__((aligned(4))) static LPDSCT_T sLPPDMA_DMIC[PDMA_RXBUFFER_CNT];

#else
    static LPDSCT_T sLPPDMA_DMIC[2];
#endif

/* Player Buffer and its pointer */
#if (NVT_DCACHE_ON == 1)
NVT_NONCACHEABLE __attribute__((aligned(4))) uint8_t PcmRecBuff[PDMA_RXBUFFER_CNT][BUFF_LEN];
#else
uint8_t PcmRecBuff[PDMA_RXBUFFER_CNT][BUFF_LEN] __attribute__((aligned(4))) = {0};
#endif
uint8_t u8PcmRxBufFull[PDMA_RXBUFFER_CNT] = {0};

volatile uint32_t u32BufRecIdx = 0;


void LPPDMA_init()
{
    uint32_t i;

    for (i = 0; i < PDMA_RXBUFFER_CNT; i++)
    {
        sLPPDMA_DMIC[i].CTL = (((u32RxBuffLen / 2) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
        sLPPDMA_DMIC[i].SA = (uint32_t)(&DMIC0->FIFO);
        sLPPDMA_DMIC[i].DA = (uint32_t) & PcmRecBuff[i];

        if (i != (PDMA_RXBUFFER_CNT - 1))
            sLPPDMA_DMIC[i].NEXT = (uint32_t)&sLPPDMA_DMIC[i + 1];
        else
            sLPPDMA_DMIC[i].NEXT = (uint32_t)&sLPPDMA_DMIC[0];

    }

    // Open LPPDMA channel
    LPPDMA_Open(LPPDMA, (1 << DMIC_LPPDMA_CH));
    // Set TransMode
    LPPDMA_SetTransferMode(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_DMIC0_RX, TRUE, (uint32_t)&sLPPDMA_DMIC[0]);
    // Enable interrupt
    LPPDMA_EnableInt(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(LPPDMA_IRQn);
    NVIC_SetPriority(LPPDMA_IRQn, 1);

}

// Push audio data to ring buffer
static int AudioInBuf_Push(S_BUF_CTRL *psBufCtrl, int16_t *pi16Data, int32_t i32PushSamples)
{
    int32_t i32NextWriteIndex;
    int32_t i32ReadIndex = psBufCtrl->i32ReadSampleIndex;
    int32_t i32WriteIndex = psBufCtrl->i32WriteSampleIndex;
    int32_t i32TotalSamples = psBufCtrl->i32TotalSamples;
    uint32_t u32Channels = psBufCtrl->u32Channels;
    int16_t *pi16AudioInBuf = psBufCtrl->pi16AudioInBuf;
    int32_t i32FreeSampleSpace;

    //buffer full, reserved a PDMA block samples to avoid overflow
    if (((i32WriteIndex + i32PushSamples) % i32TotalSamples) == i32ReadIndex)
        return -1;

    if (i32WriteIndex >= i32ReadIndex)
    {
        i32FreeSampleSpace = i32TotalSamples - (i32WriteIndex - i32ReadIndex);
    }
    else
    {
        i32FreeSampleSpace = i32ReadIndex - i32WriteIndex;
    }

    if (i32FreeSampleSpace < i32PushSamples)
        return -2;

    i32NextWriteIndex = i32WriteIndex + i32PushSamples;

    SCB_InvalidateDCache_by_Addr(pi16Data, i32PushSamples * u32Channels * sizeof(int16_t));

    if (i32NextWriteIndex >= i32TotalSamples)
    {
        int32_t i32CopySamples = i32TotalSamples - i32WriteIndex;

        memcpy(&pi16AudioInBuf[i32WriteIndex * u32Channels], pi16Data, i32CopySamples * u32Channels * sizeof(int16_t));

        i32NextWriteIndex = i32NextWriteIndex - i32TotalSamples;

        if (i32NextWriteIndex)
        {
            memcpy(&pi16AudioInBuf[0], &pi16Data[i32CopySamples * u32Channels], i32NextWriteIndex * u32Channels * sizeof(int16_t));
        }
    }
    else
    {
        memcpy(&pi16AudioInBuf[i32WriteIndex * u32Channels], pi16Data, i32PushSamples * u32Channels * sizeof(int16_t));
    }

    psBufCtrl->i32WriteSampleIndex = i32NextWriteIndex;

    return 0;
}

// Read audio data from ring buffer
static int AudioInBuf_Read(S_BUF_CTRL *psBufCtrl, int16_t *pi16Data, int32_t i32Samples)
{
    int32_t i32NextReadIndex;
    int32_t i32ReadIndex = psBufCtrl->i32ReadSampleIndex;
    int32_t i32WriteIndex = psBufCtrl->i32WriteSampleIndex;
    int32_t i32TotalSamples = psBufCtrl->i32TotalSamples;
    uint32_t u32Channels = psBufCtrl->u32Channels;
    int16_t *pi16AudioInBuf = psBufCtrl->pi16AudioInBuf;
    int32_t i32AvailSampleSpace;

    //empty
    if (i32ReadIndex == i32WriteIndex)
        return -1;

    if (i32WriteIndex > i32ReadIndex)
    {
        i32AvailSampleSpace = i32WriteIndex - i32ReadIndex;
    }
    else
    {
        i32AvailSampleSpace = i32TotalSamples - (i32ReadIndex - i32WriteIndex);
    }

    if (i32AvailSampleSpace < i32Samples)
        return -2;

    i32NextReadIndex = i32ReadIndex + i32Samples;

    if (i32NextReadIndex >= i32TotalSamples)
    {
        int32_t i32CopySamples = i32TotalSamples - i32ReadIndex;

        memcpy(pi16Data, &pi16AudioInBuf[i32ReadIndex * u32Channels],  i32CopySamples * u32Channels * sizeof(int16_t));

        i32NextReadIndex = i32NextReadIndex - i32TotalSamples;

        if (i32NextReadIndex)
        {
            memcpy(&pi16Data[i32CopySamples * u32Channels], &pi16AudioInBuf[0],  i32NextReadIndex * u32Channels * sizeof(int16_t));
        }
    }
    else
    {
        memcpy(pi16Data, &pi16AudioInBuf[i32ReadIndex * u32Channels], i32Samples * u32Channels * sizeof(int16_t));
    }

    return 0;
}

// Pop audio data from ring buffer
static int AudioInBuf_Pop(S_BUF_CTRL *psBufCtrl, int32_t i32Samples)
{
    int32_t i32NextReadIndex;
    int32_t i32ReadIndex = psBufCtrl->i32ReadSampleIndex;
    int32_t i32WriteIndex = psBufCtrl->i32WriteSampleIndex;
    int32_t i32TotalSamples = psBufCtrl->i32TotalSamples;
    int32_t i32AvailSampleSpace;

    //empty
    if (i32ReadIndex == i32WriteIndex)
        return -1;

    if (i32WriteIndex > i32ReadIndex)
    {
        i32AvailSampleSpace = i32WriteIndex - i32ReadIndex;
    }
    else
    {
        i32AvailSampleSpace = i32TotalSamples - (i32ReadIndex - i32WriteIndex);
    }

    if (i32AvailSampleSpace < i32Samples)
        return -2;

    i32NextReadIndex = i32ReadIndex + i32Samples;

    if (i32NextReadIndex >= i32TotalSamples)
    {
        i32NextReadIndex = i32NextReadIndex - i32TotalSamples;
    }

    psBufCtrl->i32ReadSampleIndex = i32NextReadIndex;

    return 0;
}

// Get available audio data size
static int AudioInBuf_AvailSamples(S_BUF_CTRL *psBufCtrl)
{
    int32_t i32ReadIndex = psBufCtrl->i32ReadSampleIndex;
    int32_t i32WriteIndex = psBufCtrl->i32WriteSampleIndex;
    int32_t i32TotalSamples = psBufCtrl->i32TotalSamples;
    int32_t i32AvailSampleSpace;

    //empty
    if (i32ReadIndex == i32WriteIndex)
        return 0;

    if (i32WriteIndex > i32ReadIndex)
    {
        i32AvailSampleSpace = i32WriteIndex - i32ReadIndex;
    }
    else
    {
        i32AvailSampleSpace = i32TotalSamples - (i32ReadIndex - i32WriteIndex);
    }

    return i32AvailSampleSpace;
}

//#define LPPDMA_PINGPONG_CHECK

#if defined(LPPDMA_PINGPONG_CHECK)
    static int s_IRQIn_index = 1;
#endif

NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32TDStatus = LPPDMA_GET_TD_STS(LPPDMA);

    if (u32TDStatus & (1 << DMIC_LPPDMA_CH))
    {
        LPPDMA_CLR_TD_FLAG(LPPDMA, (1 << DMIC_LPPDMA_CH));

        /* Set PCM buffer full flag */
        u8PcmRxBufFull[u8PDMARxIdx] = 1;
        u8RxDataCntInBuffer++;

        /* Change to next buffer */
        u8PDMARxIdx ++;

        if (u8PDMARxIdx >= PDMA_RXBUFFER_CNT)
            u8PDMARxIdx = 0;

        UAC_SendRecData();
    }
}

/**
  * @brief  SendRecData, prepare the record data for next ISO transfer.
  * @param  None.
  * @retval None.
  */
void UAC_SendRecData(void)
{
    uint32_t *pBuff;

    /* when record buffer full, send data to host */
    if (u8PcmRxBufFull[u32BufRecIdx])
    {
        /* Set empty flag */
        u8PcmRxBufFull[u32BufRecIdx] = 0;

        pBuff = (uint32_t *)&PcmRecBuff[u32BufRecIdx][0];

        /* active usbd DMA to read data to FIFO and then send to host */
        HSUSBD_SET_DMA_READ(ISO_IN_EP_NUM);
        HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk | HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
        HSUSBD_SET_DMA_ADDR((uint32_t)pBuff);
        HSUSBD_SET_DMA_LEN(u32RxBuffLen);
        g_hsusbd_DmaDone = 0;
        HSUSBD_ENABLE_DMA();

        /* Change to next PCM buffer */
        u32BufRecIdx ++;

        if (u32BufRecIdx >= PDMA_RXBUFFER_CNT)
            u32BufRecIdx = 0;

        if (u8RxDataCntInBuffer > 0)
            u8RxDataCntInBuffer --;

        AudioInBuf_Push(&s_sAudioBufCtrl, (int16_t *)pBuff, u32RxBuffLen / 2);

        /* wait usbd dma complete */
        while (1)
        {
            if (g_hsusbd_DmaDone)
                break;

            if (!HSUSBD_IS_ATTACHED())
                break;
        }

        g_usbd_txflag = 0;
    }
    else     /* send zero packet when no data*/
    {
        u8RxDataCntInBuffer = 0;
        HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EPRSPCTL_ZEROLEN_Msk;
    }
}

// Init DMIC record resource
int32_t DMICRecord_Init(
    uint32_t u32SampleRate,
    uint32_t u32Channels,
    uint32_t u32BlockSamples,
    uint32_t u32BlockCounts
)
{
    int i32Ret = 0;
    int16_t *pi16LPPDMAPingPoneBuf0 = NULL;
    int16_t *pi16LPPDMAPingPoneBuf1 = NULL;
    uint32_t u32LPPDMAPingPoneBuf0_Aligned;
    uint32_t u32LPPDMAPingPoneBuf1_Aligned;
    int16_t *pi16AudioInBuf = NULL;

    //Allocate audio-in buffers
    pi16LPPDMAPingPoneBuf0 = malloc((u32BlockSamples * u32Channels * sizeof(int16_t)) + (LPPDMA_BUF_ALIGH - 1));
    pi16LPPDMAPingPoneBuf1 = malloc((u32BlockSamples * u32Channels * sizeof(int16_t)) + (LPPDMA_BUF_ALIGH - 1));

    //Aligned LPPDMA bufffer address
    u32LPPDMAPingPoneBuf0_Aligned = (uint32_t)pi16LPPDMAPingPoneBuf0 + (LPPDMA_BUF_ALIGH - 1);
    u32LPPDMAPingPoneBuf0_Aligned &= ~(LPPDMA_BUF_ALIGH - 1);
    u32LPPDMAPingPoneBuf1_Aligned = (uint32_t)pi16LPPDMAPingPoneBuf1 + (LPPDMA_BUF_ALIGH - 1);
    u32LPPDMAPingPoneBuf1_Aligned &= ~(LPPDMA_BUF_ALIGH - 1);

    pi16AudioInBuf = malloc(u32BlockSamples * u32BlockCounts * u32Channels * sizeof(int16_t));

    if (!pi16LPPDMAPingPoneBuf0 || !pi16LPPDMAPingPoneBuf1 || !pi16AudioInBuf)
    {
        i32Ret = -1;
        goto init_fail;
    }

    DMIC_Open(DMIC0);
    // Set down sample rate 100 for quilty.(Suggest 96M used DMIC_CTL_DOWNSAMPLE_100_50 )
    DMIC_SET_DOWNSAMPLE(DMIC0, DMIC_DOWNSAMPLE_256);
    // Set DMIC sample rate.
    printf("DMIC SampleRate is %d\n", DMIC_SetSampleRate(DMIC0, u32SampleRate));

    // Enable DMIC FIFO threshold interrupt.
    DMIC_ENABLE_FIFOTH_INT(DMIC0, 16);
    // Set FIFO Width 16bits
    DMIC_SetFIFOWidth(DMIC0, DMIC_FIFOWIDTH_16);

    DMIC_ClearFIFO(DMIC0);

    while (!DMIC_IS_FIFOEMPTY(DMIC0));

    DMIC_ResetDSP(DMIC0);  //SWRST

    //DMIC Gain Setting
    DMIC_SetDSPGainVolume(DMIC0, DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk, 36);//+36dB

    // Setup MIC(RX) PDMA buffer description
    s_sLPPDMA_DMIC_SCT[0].CTL = (((u32BlockSamples * u32Channels) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_sLPPDMA_DMIC_SCT[0].SA = (uint32_t)(&DMIC0->FIFO);
    s_sLPPDMA_DMIC_SCT[0].DA = u32LPPDMAPingPoneBuf0_Aligned;
    s_sLPPDMA_DMIC_SCT[0].NEXT = (uint32_t)&s_sLPPDMA_DMIC_SCT[1];
    s_sLPPDMA_DMIC_SCT[1].CTL = (((u32BlockSamples * u32Channels) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_sLPPDMA_DMIC_SCT[1].SA = (uint32_t)(&DMIC0->FIFO);
    s_sLPPDMA_DMIC_SCT[1].DA = u32LPPDMAPingPoneBuf1_Aligned;
    s_sLPPDMA_DMIC_SCT[1].NEXT = (uint32_t)&s_sLPPDMA_DMIC_SCT[0];

    s_pi16PDMAPingPoneBuf[0] = pi16LPPDMAPingPoneBuf0;
    s_pi16PDMAPingPoneBuf[1] = pi16LPPDMAPingPoneBuf1;
    s_u32PDMAPingPoneBuf_Aligned[0] = u32LPPDMAPingPoneBuf0_Aligned;
    s_u32PDMAPingPoneBuf_Aligned[1] = u32LPPDMAPingPoneBuf1_Aligned;

    s_sAudioBufCtrl.i32ReadSampleIndex = 0;
    s_sAudioBufCtrl.i32WriteSampleIndex = 0;
    s_sAudioBufCtrl.i32TotalSamples = u32BlockSamples * u32BlockCounts;
    s_sAudioBufCtrl.u32Channels = u32Channels;
    s_sAudioBufCtrl.pi16AudioInBuf = pi16AudioInBuf;
    s_sAudioBufCtrl.u32PDMABlockSamples = u32BlockSamples;

    HSUSBD_Open(&gsHSInfo, UAC_ClassRequest, UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();
    LPPDMA_init();
    NVIC_EnableIRQ(HSUSBD_IRQn);
    HSUSBD_CLR_SE0();

    UAC_DeviceEnable(0);

    return 0;

init_fail:

    if (pi16LPPDMAPingPoneBuf0)
        free(pi16LPPDMAPingPoneBuf0);

    if (pi16LPPDMAPingPoneBuf1)
        free(pi16LPPDMAPingPoneBuf1);

    if (pi16AudioInBuf)
        free(pi16AudioInBuf);

    return i32Ret;
}

// DMIC start record
int32_t DMICRecord_StartRec(void)
{
    if (s_sAudioBufCtrl.pi16AudioInBuf != NULL)
    {
        uint32_t u32ChanMask = 0;

        if (s_sAudioBufCtrl.u32Channels == 1)
        {
            u32ChanMask = DMIC_CTL_CHEN0_Msk;
        }
        else if (s_sAudioBufCtrl.u32Channels == 2)
        {
            u32ChanMask = DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk;
        }
        else if (s_sAudioBufCtrl.u32Channels == 3)
        {
            u32ChanMask = DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk;
        }
        else if (s_sAudioBufCtrl.u32Channels == 4)
        {
            u32ChanMask = DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk;
        }

        DMIC_EnableChMsk(DMIC0, u32ChanMask);
        DMIC_ENABLE_LPPDMA(DMIC0);
    }

    return 0;
}

// DMIC stop record
int32_t DMICRecord_StopRec(void)
{
    DMIC_DISABLE_LPPDMA(DMIC0);

    uint32_t u32ChanMask = 0;

    if (s_sAudioBufCtrl.u32Channels == 1)
    {
        u32ChanMask = DMIC_CTL_CHEN0_Msk;
    }
    else if (s_sAudioBufCtrl.u32Channels == 2)
    {
        u32ChanMask = DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk;
    }
    else if (s_sAudioBufCtrl.u32Channels == 3)
    {
        u32ChanMask = DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk;
    }
    else if (s_sAudioBufCtrl.u32Channels == 4)
    {
        u32ChanMask = DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk;
    }

    DMIC_DisableChMsk(DMIC0, u32ChanMask);
    DMIC_Close(DMIC0);
    return 0;
}

// Get available audio sample number
int32_t DMICRecord_AvailSamples(void)
{
    return AudioInBuf_AvailSamples(&s_sAudioBufCtrl);
}

// Read audio sample data
int32_t DMICRecord_ReadSamples(int16_t *pi16SampleData, uint32_t u32Samples)
{
    return AudioInBuf_Read(&s_sAudioBufCtrl, pi16SampleData, u32Samples);
}

// Update audio sample data index
int32_t DMICRecord_UpdateReadSampleIndex(uint32_t u32Samples)
{
    return AudioInBuf_Pop(&s_sAudioBufCtrl, u32Samples);
}

// Un-init DMIC record
void DMICRecord_UnInit(void)
{
    DMICRecord_StopRec();

    if (s_pi16PDMAPingPoneBuf[0])
    {
        free(s_pi16PDMAPingPoneBuf[0]);
        s_pi16PDMAPingPoneBuf[0] = NULL;
    }

    if (s_pi16PDMAPingPoneBuf[1])
    {
        free(s_pi16PDMAPingPoneBuf[1]);
        s_pi16PDMAPingPoneBuf[1] = NULL;
    }

    if (s_sAudioBufCtrl.pi16AudioInBuf)
    {
        free(s_sAudioBufCtrl.pi16AudioInBuf);
        s_sAudioBufCtrl.pi16AudioInBuf = NULL;
    }

    memset(&s_sAudioBufCtrl, 0, sizeof(S_BUF_CTRL));
}
