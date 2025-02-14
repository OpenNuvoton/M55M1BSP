/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   This sample uses DMIC as audio input(MIC) and I2S as audio output(SPK).
 *          User can process audio data before output.
 *          Data have been transfered via PDMA/LPPDMA.
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This sample uses internal RC as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * Debug UART
 *   system_M55M1.c has three weak functions as below to configure debug UART port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global constants                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define NAU8822            1
#define I2C_PORT           I2C3
#define SAMPLE_RATE        (16000)
#define BUF_COUNT          (256)
#define DMIC_LPPDMA_CH     (3)
#define I2S0TX_PDMA_CH     (2)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void I2STX_Init(void);
void I2STX_Start(void);
void I2STX_Stop(void);
void DMIC_Init(void);
void DMIC_Start(void);
void DMIC_Stop(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t s_u8WriteBufIdx, s_u8LppdmaBusy; // Buffer control handler.
static int32_t s_SampleRate;
volatile uint8_t g_u8PCMBufferFull[2] = {0, 0};
volatile uint8_t g_u8PCMBufferPlaying = 0;
volatile uint8_t u8AudioPlaying = 0;

// Provide PDMA description for ping-pong.
#if (NVT_DCACHE_ON == 1)
    // If DCACHE is enabled, use a cache-line aligned buffer for the I2S PCM DMA
    signed int aiPCMBuffer[DCACHE_ALIGN_LINE_SIZE(2)][DCACHE_ALIGN_LINE_SIZE(BUF_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    /* DMA descriptor table, must be aligned to 32 bytes and placed in DTCM */
    NVT_DTCM __ALIGNED(32) static DSCT_T sPDMA_I2STX[2];
#else
    // If DCACHE is not enabled, use a standard buffer for the I2S PCM DMA
    signed int aiPCMBuffer[2][BUF_COUNT];
    /* DMA descriptor table, must be aligned to 32 bytes */
    static DSCT_T sPDMA_I2STX[2] __attribute__((aligned(32)));
#endif

// LPPMDA =========================================================================================================
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32TDStatus = LPPDMA_GET_TD_STS(LPPDMA);

    if (u32TDStatus & (1 << DMIC_LPPDMA_CH))
    {
        LPPDMA_CLR_TD_FLAG(LPPDMA, (1 << DMIC_LPPDMA_CH));

        g_u8PCMBufferFull[s_u8WriteBufIdx] = 1;
    }

    s_u8LppdmaBusy = 0;
    __DSB();
    __ISB();

    while (LPPDMA_GET_TD_STS(LPPDMA) || s_u8LppdmaBusy)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA IntFlag time-out!\n");
        }
    }
}

// PMDA =========================================================================================================
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32TDStatus = PDMA_GET_TD_STS(PDMA0);

    if (u32TDStatus & (1 << I2S0TX_PDMA_CH))
    {
        PDMA_CLR_TD_FLAG(PDMA0, (1 << I2S0TX_PDMA_CH));

        g_u8PCMBufferFull[g_u8PCMBufferPlaying] = 0;       /* Set empty flag */
        g_u8PCMBufferPlaying ^= 1;
    }

    __DSB();
    __ISB();

    while (PDMA_GET_TD_STS(PDMA0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA0 IntFlag time-out!\n");
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
void DMIC_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable PLL1 192MHZ clock from HIRC for DMIC clock source to PLL1_DIV2 */
    printf("Get real APLL1 Frequency is %d\n", CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_192MHZ, CLK_APLL1_SELECT));
    // Select DMIC CLK source from PLL1_DIV2.
    CLK_SetModuleClock(DMIC0_MODULE, CLK_DMICSEL_DMIC0SEL_APLL1_DIV2, MODULE_NoMsk);
    // Enable DMIC clock.
    CLK_EnableModuleClock(DMIC0_MODULE);
    // DPWM IPReset.
    SYS_ResetModule(SYS_DMIC0RST);
    /* Lock protected registers */
    SYS_LockReg();

    DMIC_Open(DMIC0);
    // FREQ_192MHZ for DMIC 8000/16000/48000 Hz sample-rate with 50/100 down-sample
    DMIC_SET_DOWNSAMPLE(DMIC0, DMIC_DOWNSAMPLE_100);
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
    //DMIC_ENABLE_FIFOTH_INT(DMIC0, 16);
    // Set FIFO Width 16bits
    DMIC_SetFIFOWidth(DMIC0, DMIC_FIFOWIDTH_16);

    DMIC_ClearFIFO(DMIC0);

    while (!DMIC_IS_FIFOEMPTY(DMIC0));

    DMIC_ResetDSP(DMIC0);  //SWRST

    //    //DMIC Gain Setting
    //    DMIC_SetDSPGainVolume(DMIC0, DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk, 36);//+36dB

    // Open LPPDMA channel
    LPPDMA_Open(LPPDMA, (1 << DMIC_LPPDMA_CH));
    /* Transfer count is BUF_COUNT*2, transfer width is 16 bits(one PCM) */
    LPPDMA_SetTransferCnt(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_WIDTH_16, BUF_COUNT * 2);
    /* Set source address is DMIC0->FIFO, destination address is aiPCMBuffer, Source/Destination increment size is 32 bits(one word) */
    LPPDMA_SetTransferAddr(LPPDMA, DMIC_LPPDMA_CH, (uint32_t)(&DMIC0->FIFO), LPPDMA_SAR_FIX, (uint32_t) & (aiPCMBuffer[0][0]), LPPDMA_DAR_INC);
    /* Request source is DMIC0_RX to memory */
    LPPDMA_SetTransferMode(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_DMIC0_RX, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 1 */
    LPPDMA_SetBurstType(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    // Enable interrupt
    LPPDMA_EnableInt(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_INT_TRANS_DONE);
    // GPIO multi-function.
    SET_DMIC0_DAT_PB5();
    SET_DMIC0_CLK_PB4();
}

void DMIC_Start(void)
{
    DMIC_EnableChMsk(DMIC0, DMIC_CTL_CHEN0_Msk);
    DMIC_ENABLE_LPPDMA(DMIC0);
}

void DMIC_Stop(void)
{
    DMIC_DISABLE_LPPDMA(DMIC0);
    DMIC_Close(DMIC0);
}

// Speaker(DPWM) = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
void I2STX_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    // Select I2S CLK source from APLL1_DIV2.
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_APLL1_DIV2, CLK_I2SDIV_I2S0DIV(8));

    // Enable I2S clock.
    CLK_EnableModuleClock(I2S0_MODULE);
    // I2S IPReset.
    SYS_ResetModule(SYS_I2S0RST);
    /* Lock protected registers */
    SYS_LockReg();
    // Open I2S0 hardware IP
    /* Configure as I2S slave */
    s_SampleRate = I2S_Open(I2S0, I2S_MODE_SLAVE, SAMPLE_RATE, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S);
    printf("I2S SampleRate is %d\n", s_SampleRate);
    //Enable MCLK
    printf("I2S MCLK is %d\n", I2S_EnableMCLK(I2S0, 12000000));
    // I2S0 Configuration
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;

    printf("  DMIC to I2S0 through FIFO test\n");

    // Clear TX FIFO buffer
    I2S_CLR_TX_FIFO(I2S0);

    // SPK(TX) buffer description
    sPDMA_I2STX[0].CTL = ((BUF_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    sPDMA_I2STX[0].SA = (uint32_t)&aiPCMBuffer[0][0];
    sPDMA_I2STX[0].DA = (uint32_t)(&I2S0->TXFIFO);
    sPDMA_I2STX[0].NEXT = (uint32_t)&sPDMA_I2STX[1];
    sPDMA_I2STX[1].CTL = ((BUF_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    sPDMA_I2STX[1].SA = (uint32_t)&aiPCMBuffer[1][0];
    sPDMA_I2STX[1].DA = (uint32_t)(&I2S0->TXFIFO);
    sPDMA_I2STX[1].NEXT = (uint32_t)&sPDMA_I2STX[0];
    // Open PDMA channel
    PDMA_Open(PDMA0, (1 << I2S0TX_PDMA_CH));
    // Set TransMode
    PDMA_SetTransferMode(PDMA0, I2S0TX_PDMA_CH, PDMA_I2S0_TX, TRUE, (uint32_t)&sPDMA_I2STX[0]);
    // Enable interrupt
    PDMA_EnableInt(PDMA0, I2S0TX_PDMA_CH, PDMA_INT_TRANS_DONE);

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;
}

void I2STX_Start(void)
{
    I2S_ENABLE_TXDMA(I2S0);
    I2S_ENABLE_TX(I2S0);
}

void I2STX_Stop(void)
{
    I2S_DISABLE_TX(I2S0);
    I2S_DISABLE_TXDMA(I2S0);
}

#if NAU8822
/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C                                         */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data)
{
    I2C_START(I2C_PORT);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)((u8Addr << 1) | (u16Data >> 8)));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16Data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_STOP(I2C_PORT);
}

/* Config play sampling rate */
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU8822] Configure Sampling Rate to %d\n", u32SampleRate);

    if ((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU8822(36, 0x008);    //12.288Mhz
        I2C_WriteNAU8822(37, 0x00C);
        I2C_WriteNAU8822(38, 0x093);
        I2C_WriteNAU8822(39, 0x0E9);
    }
    else
    {
        I2C_WriteNAU8822(36, 0x007);    //11.2896Mhz
        I2C_WriteNAU8822(37, 0x021);
        I2C_WriteNAU8822(38, 0x161);
        I2C_WriteNAU8822(39, 0x026);
    }

    switch (u32SampleRate)
    {
        case 16000:
            I2C_WriteNAU8822(6, 0x1AD);    /* Divide by 6, 16K */
            I2C_WriteNAU8822(7, 0x006);    /* 16K for internal filter coefficients */
            break;

        case 44100:
            I2C_WriteNAU8822(6, 0x14D);    /* Divide by 2, 48K */
            I2C_WriteNAU8822(7, 0x000);    /* 48K for internal filter coefficients */
            break;

        case 48000:
            I2C_WriteNAU8822(6, 0x14D);    /* Divide by 2, 48K */
            I2C_WriteNAU8822(7, 0x000);    /* 48K for internal filter coefficients */
            break;

        case 96000:
            I2C_WriteNAU8822(6, 0x109);    /* Divide by 1, 96K */
            I2C_WriteNAU8822(72, 0x013);
            break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K */
    I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    printf("[OK]\n");
}

#else   // NAU88L25
uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len)
{
    (void)u32Len;

    /* Send START */
    I2C_START(I2C_PORT);
    I2C_WAIT_READY(I2C_PORT);

    /* Send device address */
    I2C_SET_DATA(I2C_PORT, u8ChipAddr);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16SubAddr >> 8));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16SubAddr & 0x00FF));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send data */
    I2C_SET_DATA(I2C_PORT, p[0]);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send data */
    I2C_SET_DATA(I2C_PORT, p[1]);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send STOP */
    I2C_STOP(I2C_PORT);

    return  0;
}

uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat)
{
    uint8_t u8TxData0[2];

    u8TxData0[0] = (uint8_t)(u16Dat >> 8);
    u8TxData0[1] = (uint8_t)(u16Dat & 0x00FF);

    return (I2C_WriteMultiByteforNAU88L25(0x1A << 1, u16Addr, &u8TxData0[0], 2));
}

/* Config play sampling rate */
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU88L25] Configure Sampling Rate to %d\n", u32SampleRate);

    if ((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU88L25(0x0005, 0x3126); //12.288Mhz
        I2C_WriteNAU88L25(0x0006, 0x0008);
    }
    else
    {
        I2C_WriteNAU88L25(0x0005, 0x86C2); //11.2896Mhz
        I2C_WriteNAU88L25(0x0006, 0x0007);
    }

    switch (u32SampleRate)
    {
        case 16000:
            I2C_WriteNAU88L25(0x0003,  0x801B); /* MCLK = SYSCLK_SRC/12 */
            I2C_WriteNAU88L25(0x0004,  0x0001);
            I2C_WriteNAU88L25(0x0005,  0x3126); /* MCLK = 4.096MHz */
            I2C_WriteNAU88L25(0x0006,  0x0008);
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=MCLK/8=512K, LRC_DIV=512K/32=16K */
            I2C_WriteNAU88L25(0x002B,  0x0002);
            I2C_WriteNAU88L25(0x002C,  0x0082);
            break;

        case 44100:
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=11.2896M/8=1.4112M, LRC_DIV=1.4112M/32=44.1K */
            I2C_WriteNAU88L25(0x002B,  0x0012);
            I2C_WriteNAU88L25(0x002C,  0x0082);
            break;

        case 48000:
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K */
            I2C_WriteNAU88L25(0x002B,  0x0012);
            I2C_WriteNAU88L25(0x002C,  0x0082);
            break;

        case 96000:
            I2C_WriteNAU88L25(0x0003,  0x80A2); /* MCLK = SYSCLK_SRC/2 */
            I2C_WriteNAU88L25(0x0004,  0x1801);
            I2C_WriteNAU88L25(0x0005,  0x3126); /* MCLK = 24.576MHz */
            I2C_WriteNAU88L25(0x0006,  0xF008);
            I2C_WriteNAU88L25(0x001D,  0x301A); /* 301A:Master, BCLK_DIV=MCLK/8=3.072M, LRC_DIV=3.072M/32=96K */
            I2C_WriteNAU88L25(0x002B,  0x0001);
            I2C_WriteNAU88L25(0x002C,  0x0080);
            break;
    }
}

void NAU88L25_Reset(void)
{
    I2C_WriteNAU88L25(0,  0x1);
    I2C_WriteNAU88L25(0,  0);   /* Reset all registers */
    CLK_SysTickDelay(10000);

    printf("NAU88L25 Software Reset.\n");
}

void NAU88L25_Setup(void)
{
    I2C_WriteNAU88L25(0x0003,  0x8053);
    I2C_WriteNAU88L25(0x0004,  0x0001);
    I2C_WriteNAU88L25(0x0005,  0x3126);
    I2C_WriteNAU88L25(0x0006,  0x0008);
    I2C_WriteNAU88L25(0x0007,  0x0010);
    I2C_WriteNAU88L25(0x0008,  0xC000);
    I2C_WriteNAU88L25(0x0009,  0x6000);
    I2C_WriteNAU88L25(0x000A,  0xF13C);
    I2C_WriteNAU88L25(0x000C,  0x0048);
    I2C_WriteNAU88L25(0x000D,  0x0000);
    I2C_WriteNAU88L25(0x000F,  0x0000);
    I2C_WriteNAU88L25(0x0010,  0x0000);
    I2C_WriteNAU88L25(0x0011,  0x0000);
    I2C_WriteNAU88L25(0x0012,  0xFFFF);
    I2C_WriteNAU88L25(0x0013,  0x0015);
    I2C_WriteNAU88L25(0x0014,  0x0110);
    I2C_WriteNAU88L25(0x0015,  0x0000);
    I2C_WriteNAU88L25(0x0016,  0x0000);
    I2C_WriteNAU88L25(0x0017,  0x0000);
    I2C_WriteNAU88L25(0x0018,  0x0000);
    I2C_WriteNAU88L25(0x0019,  0x0000);
    I2C_WriteNAU88L25(0x001A,  0x0000);
    I2C_WriteNAU88L25(0x001B,  0x0000);
    I2C_WriteNAU88L25(0x001C,  0x0002);
    I2C_WriteNAU88L25(0x001D,  0x301A);   /* 301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K */
    I2C_WriteNAU88L25(0x001E,  0x0000);
    I2C_WriteNAU88L25(0x001F,  0x0000);
    I2C_WriteNAU88L25(0x0020,  0x0000);
    I2C_WriteNAU88L25(0x0021,  0x0000);
    I2C_WriteNAU88L25(0x0022,  0x0000);
    I2C_WriteNAU88L25(0x0023,  0x0000);
    I2C_WriteNAU88L25(0x0024,  0x0000);
    I2C_WriteNAU88L25(0x0025,  0x0000);
    I2C_WriteNAU88L25(0x0026,  0x0000);
    I2C_WriteNAU88L25(0x0027,  0x0000);
    I2C_WriteNAU88L25(0x0028,  0x0000);
    I2C_WriteNAU88L25(0x0029,  0x0000);
    I2C_WriteNAU88L25(0x002A,  0x0000);
    I2C_WriteNAU88L25(0x002B,  0x0012);
    I2C_WriteNAU88L25(0x002C,  0x0082);
    I2C_WriteNAU88L25(0x002D,  0x0000);
    I2C_WriteNAU88L25(0x0030,  0x00CF);
    I2C_WriteNAU88L25(0x0031,  0x0000);
    I2C_WriteNAU88L25(0x0032,  0x0000);
    I2C_WriteNAU88L25(0x0033,  0x009E);
    I2C_WriteNAU88L25(0x0034,  0x029E);
    I2C_WriteNAU88L25(0x0038,  0x1486);
    I2C_WriteNAU88L25(0x0039,  0x0F12);
    I2C_WriteNAU88L25(0x003A,  0x25FF);
    I2C_WriteNAU88L25(0x003B,  0x3457);
    I2C_WriteNAU88L25(0x0045,  0x1486);
    I2C_WriteNAU88L25(0x0046,  0x0F12);
    I2C_WriteNAU88L25(0x0047,  0x25F9);
    I2C_WriteNAU88L25(0x0048,  0x3457);
    I2C_WriteNAU88L25(0x004C,  0x0000);
    I2C_WriteNAU88L25(0x004D,  0x0000);
    I2C_WriteNAU88L25(0x004E,  0x0000);
    I2C_WriteNAU88L25(0x0050,  0x2007);
    I2C_WriteNAU88L25(0x0051,  0x0000);
    I2C_WriteNAU88L25(0x0053,  0xC201);
    I2C_WriteNAU88L25(0x0054,  0x0C95);
    I2C_WriteNAU88L25(0x0055,  0x0000);
    I2C_WriteNAU88L25(0x0058,  0x1A14);
    I2C_WriteNAU88L25(0x0059,  0x00FF);
    I2C_WriteNAU88L25(0x0066,  0x0060);
    I2C_WriteNAU88L25(0x0068,  0xC300);
    I2C_WriteNAU88L25(0x0069,  0x0000);
    I2C_WriteNAU88L25(0x006A,  0x0083);
    I2C_WriteNAU88L25(0x0071,  0x0011);
    I2C_WriteNAU88L25(0x0072,  0x0260);
    I2C_WriteNAU88L25(0x0073,  0x332C);
    I2C_WriteNAU88L25(0x0074,  0x4502);
    I2C_WriteNAU88L25(0x0076,  0x3140);
    I2C_WriteNAU88L25(0x0077,  0x0000);
    I2C_WriteNAU88L25(0x007F,  0x553F);
    I2C_WriteNAU88L25(0x0080,  0x0420);
    I2C_WriteNAU88L25(0x0001,  0x07D4);

    printf("NAU88L25 Configured done.\n");
}
#endif

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);
    SYS_ResetModule(SYS_I2C3RST);

    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);

    // PDMA/LPPDMA Initial.
    // Enable PDMA clock.
    CLK_EnableModuleClock(PDMA0_MODULE);
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    // Reset PDMA module
    SYS_ResetModule(SYS_PDMA0RST);
    SYS_ResetModule(SYS_LPPDMA0RST);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("DMIC as audio input(MIC) and I2S as audio output(Codec).\n");

    /* Init I2C to access codec */
    I2C_Init();

#if (!NAU8822)
    /* Reset NAU88L25 codec */
    NAU88L25_Reset();
#endif

    /* Set JK-EN low to enable phone jack on NuMaker board. */
    SET_GPIO_PD1();
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT);
    PD1 = 0;

#if NAU8822
    /* Initialize NAU8822 codec */
    NAU8822_Setup();
    NAU8822_ConfigSampleRate(SAMPLE_RATE);
#else
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    // Enable PDMA's NVIC
    NVIC_EnableIRQ(PDMA0_IRQn);
    NVIC_EnableIRQ(LPPDMA_IRQn);

    s_u8WriteBufIdx = 0;
    s_u8LppdmaBusy = 0;
    // Initiate microphone.
    DMIC_Init();
    // Start microphone.
    s_u8LppdmaBusy = 1;
    DMIC_Start();
    // Initiate speaker.
    I2STX_Init();

    // while loop for processing.
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
                // Start speaker.
                I2STX_Start();
                printf("Start Playing ...\n");
            }

            while ((g_u8PCMBufferFull[0] == 1) && (g_u8PCMBufferFull[1] == 1));
        }

        if (s_u8LppdmaBusy == 0)
        {
            s_u8LppdmaBusy = 1;
            s_u8WriteBufIdx ^= 1;

            /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
            if (s_u8WriteBufIdx)
                LPPDMA_SetTransferAddr(LPPDMA, DMIC_LPPDMA_CH, (uint32_t)(&DMIC0->FIFO), LPPDMA_SAR_FIX, (uint32_t)(&aiPCMBuffer[1][0]), LPPDMA_DAR_INC);
            else
                LPPDMA_SetTransferAddr(LPPDMA, DMIC_LPPDMA_CH, (uint32_t)(&DMIC0->FIFO), LPPDMA_SAR_FIX, (uint32_t)(&aiPCMBuffer[0][0]), LPPDMA_DAR_INC);

            /* Transfer count is BUF_COUNT*2, transfer width is 16 bits(one PCM) */
            LPPDMA_SetTransferCnt(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_WIDTH_16, BUF_COUNT * 2);
            LPPDMA_SetTransferMode(LPPDMA, DMIC_LPPDMA_CH, LPPDMA_DMIC0_RX, FALSE, 0);
        }
    };
}
