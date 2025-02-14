/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   This sample uses DMIC as audio input(MIC) and record to WAV file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This sample uses internal RC (HIRC) as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * Debug UART
 *   system_M55M1.c has three weak functions as below to configure debug port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "BUFCTRL.h"
#include "WavFileUtil.h"
#include "ff.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global constants                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define SAMPLE_RATE              (8000)
#define BUF_COUNT                (16)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void DMIC_Init(S_BUFCTRL *psInBufCtrl);
void DMIC_Start(void);
void DMIC_Stop(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile S_BUFCTRL sInBufCtrl;//, sOutBufCtrl; // Buffer control handler.
int32_t s_SampleRate;     // Buffer array: store audio data receiced from DMIC(4Channel)
char s_strFileName[100];

#define DMIC_LPPDMA_CH       (2)

S_BUFCTRL *psDMIC_BufCtrl = NULL;           // Provide microphone input buffer control.
// Provide LPPDMA description for ping-pong.
#if (NVT_DCACHE_ON == 1)
    /*
    The setting value of Descript Table is fixed, so it is placed in the non-cacheable section
    , eliminating the need for cache operations.
    */
    NVT_NONCACHEABLE __attribute__((aligned(4))) static LPDSCT_T sLPPDMA_DMIC[2];
    // If DCACHE is enabled, use a cache-line aligned buffer for the DMIC PCM DMA
    int32_t ai32InBuf[DCACHE_ALIGN_LINE_SIZE(BUF_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // Placing the DMA descriptor table in normal SRAM.
    static LPDSCT_T sLPPDMA_DMIC[2];
    // If DCACHE is not enabled, use a standard buffer for the DMIC PCM DMA
    int32_t ai32InBuf[BUF_COUNT];
#endif

NVT_ITCM void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;

    // FMI data abort interrupt
    if (SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    ier = SDH0->INTEN;

    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
        //printf("SD block down\r\n");
    }

    if ((ier & SDH_INTEN_CDIEN_Msk) &&
            (isr & SDH_INTSTS_CDIF_Msk))    // card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK

            for (i = 0; i < 0x500; i++); // delay to make sure got updated value from REG_SDISR.

            isr = SDH0->INTSTS;
        }

#if (DEF_CARD_DETECT_SOURCE == CardDetect_From_DAT3)

        if (!(isr & SDH_INTSTS_CDSTS_Msk))
#else
        if (isr & SDH_INTSTS_CDSTS_Msk)
#endif
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            //memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            //SDH_Open(SDH0, CardDetect_From_GPIO);
            //SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!SD0.R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }

        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }

    __DSB();
    __ISB();
}

// LPPMDA =========================================================================================================
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32TDStatus = LPPDMA_GET_TD_STS(LPPDMA);

    if (u32TDStatus & (1 << DMIC_LPPDMA_CH))
    {
        LPPDMA_CLR_TD_FLAG(LPPDMA, (1 << DMIC_LPPDMA_CH));
        psDMIC_BufCtrl->u16DataCount += (psDMIC_BufCtrl->u16BufCount / 2);

        if ((psDMIC_BufCtrl->u16WriteIdx += (psDMIC_BufCtrl->u16BufCount / 2)) >= psDMIC_BufCtrl->u16BufCount)
            psDMIC_BufCtrl->u16WriteIdx = 0;
    }

    __DSB();
    __ISB();

    while (LPPDMA_GET_TD_STS(LPPDMA))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA IntFlag time-out!\n");
        }
    }
}

/*-----------------------------------------------------------------
  | APLL1                     | Sample-Rate Hz      | Down-Sample |
  |---------------------------|---------------------|-------------|
  | FREQ_192MHZ               | 8000/16000/48000 Hz | 50/100      |
  |---------------------------|---------------------|-------------|
  | DMIC_APLL1_FREQ_196608KHZ | 8000/16000/48000 Hz | 64/128/256  |
  |---------------------------|---------------------|-------------|
  | DMIC_APLL1_FREQ_194040KHZ | 11025/22050/44100 Hz| 50/100      |
  |---------------------------|---------------------|-------------|
  | DMIC_APLL1_FREQ_180634KHZ | 11025/22050/44100 Hz| 64/128/256  |
  ---------------------------------------------------------------*/
// Microphone(DMIC)= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
void DMIC_Init(S_BUFCTRL *psInBufCtrl)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    // DMIC_APLL1_FREQ_196608KHZ for DMIC 8000/16000/48000 Hz sample-rate with 64/128/256 down-sample
    printf("Get real APLL1 Frequency is %d\n", CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, DMIC_APLL1_FREQ_196608KHZ, CLK_APLL1_SELECT));
    // Select DMIC CLK source from APLL1_DIV2.
    CLK_SetModuleClock(DMIC0_MODULE, CLK_DMICSEL_DMIC0SEL_APLL1_DIV2, MODULE_NoMsk);
    // Enable DMIC clock.
    CLK_EnableModuleClock(DMIC0_MODULE);
    // DPWM IPReset.
    SYS_ResetModule(SYS_DMIC0RST);
    /* Lock protected registers */
    SYS_LockReg();

    DMIC_Open(DMIC0);
    // DMIC_APLL1_FREQ_196608KHZ for DMIC 8000/16000/48000 Hz sample-rate with 64/128/256 down-sample
    DMIC_SET_DOWNSAMPLE(DMIC0, DMIC_DOWNSAMPLE_256);
    // Set DMIC sample rate.
    s_SampleRate = DMIC_SetSampleRate(DMIC0, SAMPLE_RATE);
    printf("DMIC SampleRate is %d\n", s_SampleRate);
    // Set channel's latch data falling type.
    //DMIC_SET_LATCHEDGE_CH01(DMIC0, DMIC_LATCHDATA_CH01FR);
    //DMIC_SET_LATCHEDGE_CH23(DMIC0, DMIC_LATCHDATA_CH23FR);
    //Gain step
    //DMIC_SetGainStep(DMIC0, DMIC_GAINSTEP_1_2);
    // MUTE control
    //DMIC_EnableMute(DMIC0, DMIC_CTL_CH0MUTE_Msk|DMIC_CTL_CH1MUTE_Msk|DMIC_CTL_CH2MUTE_Msk|DMIC_CTL_CH3MUTE_Msk);
    // Enable DMIC FIFO threshold interrupt.
    DMIC_ENABLE_FIFOTH_INT(DMIC0, 16);
    // Set FIFO Width 16bits
    DMIC_SetFIFOWidth(DMIC0, DMIC_FIFOWIDTH_16);

    DMIC_ClearFIFO(DMIC0);

    while (!DMIC_IS_FIFOEMPTY(DMIC0));

    DMIC_ResetDSP(DMIC0);  //SWRST

    //DMIC Gain Setting
    DMIC_SetDSPGainVolume(DMIC0, DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk, 36);//+36dB

    // MIC(RX) buffer description
    sLPPDMA_DMIC[0].CTL = ((psInBufCtrl->u16BufCount - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    sLPPDMA_DMIC[0].SA = (uint32_t)(&DMIC0->FIFO);
    sLPPDMA_DMIC[0].DA = (uint32_t) & (psInBufCtrl->pai32Buf[0]);
    sLPPDMA_DMIC[0].NEXT = (uint32_t)&sLPPDMA_DMIC[1];
    sLPPDMA_DMIC[1].CTL = ((psInBufCtrl->u16BufCount - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    sLPPDMA_DMIC[1].SA = (uint32_t)(&DMIC0->FIFO);
    sLPPDMA_DMIC[1].DA = (uint32_t) & (psInBufCtrl->pai32Buf[psInBufCtrl->u16BufCount / 2]);
    sLPPDMA_DMIC[1].NEXT = (uint32_t)&sLPPDMA_DMIC[0];
    // Open LPPDMA channel
    LPPDMA_Open(LPPDMA, (1 << DMIC_LPPDMA_CH));
    // Set TransMode
    LPPDMA_SetTransferMode(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_DMIC0_RX, TRUE, (uint32_t)&sLPPDMA_DMIC[0]);
    // Enable interrupt
    LPPDMA_EnableInt(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_INT_TRANS_DONE);
    // GPIO multi-function.
    SET_DMIC0_DAT_PB5();
    SET_DMIC0_CLK_PB4();
    // Config DMIC buffer control
    psDMIC_BufCtrl = psInBufCtrl;
}

void DMIC_Start(void)
{
    if (psDMIC_BufCtrl != NULL)
    {
        DMIC_EnableChMsk(DMIC0, DMIC_CTL_CHEN0_Msk);
        DMIC_ENABLE_LPPDMA(DMIC0);
    }
}

void DMIC_Stop(void)
{
    DMIC_DISABLE_LPPDMA(DMIC0);
    DMIC_Close(DMIC0);
}

S_WavFileWriteInfo  sWavFileInfo;

// Close WAV media
static int32_t WAVWrite_Close(void)
{
    // Finalize WavFileUtil
    if (!WavFileUtil_Write_Finish(&sWavFileInfo))
    {
        printf("WavFileUtil_Write_Finish failed\n");
        return -3;
    } // if

    printf("Close Done:%s\n\n\n", s_strFileName);
    return 0;
} // WAVWrite_Close()

static int32_t WAVWrite_Open(void)
{
    // Initialize WavFileUtil
    S_WavFileWriteInfo  *psWavFileInfo = &sWavFileInfo;
    E_WavFormatTag      eFormatTag;
    int32_t             eError = 0;
    UINT16                  u16BitsPerSample;
    UINT32              u32ChannelNum = 1, u32SampleRate = s_SampleRate;

    printf("Open %s\n", s_strFileName);

    if (!WavFileUtil_Write_Initialize(psWavFileInfo, (PCSTR) s_strFileName))
    {
        eError = -1;
        goto EndOfOpen;
    } // if

    eFormatTag = eWAVE_FORMAT_PCM;
    u16BitsPerSample = 16;  // libnmedia only supports PCM_S16LE

    if (!WavFileUtil_Write_SetFormat(psWavFileInfo, eFormatTag,
                                     u32ChannelNum, u32SampleRate,
                                     u16BitsPerSample))
    {
        eError = -2;
        goto EndOfOpen;
    } // if

EndOfOpen:

    if (eError != 0)
        // Finalize WavFileUtil and free media resource
        WAVWrite_Close();
    else
        printf("Open Done:%s\n", s_strFileName);

    return eError;
} // WAVWrite_Open()

// Write chunk
// (Application should do sync on demand by itself)
static int32_t WAVWrite_WriteChunk(const BYTE *pDataVAddr, uint32_t u32DataSize)
{
    if (u32DataSize > 0)
    {
        if (pDataVAddr == 0)
            return -4;

        if (!WavFileUtil_Write_WriteData(&sWavFileInfo, pDataVAddr, u32DataSize))
        {
            printf("WavFileUtil_Write_WriteData(%uB) failed\n", u32DataSize);
            return -5;
        } // if
    } // if

    return 0;
} // WAVWrite_WriteChunk()

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL1 192MHZ clock from HIRC for DMIC clock source to PLL1_DIV2 */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_192MHZ, CLK_APLL1_SELECT);
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable SDH0 module clock source as HCLK and SDH0 module clock divider as 4 */
    CLK_SetModuleClock(SDH0_MODULE, CLK_SDHSEL_SDH0SEL_HCLK0, CLK_SDHDIV_SDH0DIV(4));
    CLK_EnableModuleClock(SDH0_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for SDH */
    SET_SD0_nCD_PD13();
    SET_SD0_CLK_PE6();
    SET_SD0_CMD_PE7();
    SET_SD0_DAT0_PE2();
    SET_SD0_DAT1_PE3();
    SET_SD0_DAT2_PE4();
    SET_SD0_DAT3_PE5();

    // LPPDMA Initial.
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    SYS_ResetModule(SYS_LPPDMA0RST);
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    int32_t i32Data[4], i32FileNameidx = 0;
    uint32_t u32DataCnt = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);

    printf("DMIC as audio input(MIC) and record to WAV file.\n");

    // Enable PDMA's NVIC
    NVIC_EnableIRQ(LPPDMA_IRQn);

    /* Configure FATFS */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* set default path */

    // These defines are from  BUFCTRL.h for buffer control in this samle.
    // Buffer control handler configuration.
    BUFCTRL_CFG((&sInBufCtrl), ai32InBuf, sizeof(ai32InBuf) / sizeof(uint32_t));

    // Initiate microphone.
    DMIC_Init((S_BUFCTRL *)&sInBufCtrl);

    while (1)
    {
        printf("press any key to Recorder!! or Esc to exit\n");
        char ch = getchar();

        if (ch == 27) //Esc to exit
            break;

        sprintf(s_strFileName, "0:\\DMIC%d.wav", i32FileNameidx++);

        // Initiate WAV.
        if (WAVWrite_Open())
            break;

#if (NVT_DCACHE_ON == 1)
        /*
           Clean the CPU Data cache before starting the DMA transfer.
           This guarantees that the source buffer will be up to date before starting the transfer.
        */
        SCB_CleanDCache_by_Addr(ai32InBuf, sizeof(ai32InBuf));
#endif  // (NVT_DCACHE_ON == 1)
        // These defines are from  BUFCTRL.h for buffer control in this samle.
        // Buffer control handler configuration.
        BUFCTRL_CFG((&sInBufCtrl), ai32InBuf, sizeof(ai32InBuf) / sizeof(uint32_t));

        u32DataCnt = 0;
        // Start microphone.
        DMIC_Start();

        // while loop for processing.
        while (u32DataCnt < 16 * s_SampleRate)
        {
            while (BUFCTRL_GET_COUNT((&sInBufCtrl)) >= 4)
            {
#if (NVT_DCACHE_ON == 1)
                /*
                   Invalidate the CPU Data cache after the DMA transfer.
                   As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
                */
                SCB_InvalidateDCache_by_Addr(ai32InBuf, sizeof(ai32InBuf));
#endif  // (NVT_DCACHE_ON == 1)

                BUFCTRL_READ((&sInBufCtrl), &i32Data[0]);
                BUFCTRL_READ((&sInBufCtrl), &i32Data[1]);
                BUFCTRL_READ((&sInBufCtrl), &i32Data[2]);
                BUFCTRL_READ((&sInBufCtrl), &i32Data[3]);
#if (NVT_DCACHE_ON == 1)
                /*
                   Clean the CPU Data cache before starting the DMA transfer.
                   This guarantees that the source buffer will be up to date before starting the transfer.
                */
                SCB_CleanDCache_by_Addr(ai32InBuf, sizeof(ai32InBuf));
#endif  // (NVT_DCACHE_ON == 1)
                // WAV Write.
                WAVWrite_WriteChunk((const BYTE *)i32Data, 16);
                u32DataCnt += 8;
            }
        };

        DMIC_Stop();

        WAVWrite_Close();

    }

    printf("DMIC WAV record done.\n");
    DMIC_Stop();
    WAVWrite_Close();
    SDH_Close_Disk(SDH0);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
