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
#include "BUFCTRL.h"

#define NAU8822     1

//------------------------------------------------------------------------------
#if defined(ALIGN_AF_PINS)
    #define I2C_PORT                        I2C3
#else
    #define I2C_PORT                        I2C2
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Define global constants                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define SAMPLE_RATE              (16000)
#define BUF_COUNT                (16)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void I2STX_Init(S_BUFCTRL *psOutBufCtrl);
void I2STX_Start(void);
void I2STX_Stop(void);
void DMIC_Init(S_BUFCTRL *psInBufCtrl);
void DMIC_Start(void);
void DMIC_Stop(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile S_BUFCTRL sInBufCtrl, sOutBufCtrl; // Buffer control handler.
int32_t    ai32InBuf[BUF_COUNT], s_SampleRate;      // Buffer array: store audio data receiced from DMIC(4Channel)
int32_t    ai32OutBuf[BUF_COUNT];         // Buffer array: store audio data ready to send to DPWM(2Channel)

#define DMIC_LPPDMA_CH       (2)

S_BUFCTRL *psDMIC_BufCtrl = NULL;           // Provide microphone input buffer control.
LPDSCT_T     sLPPDMA_DMIC[2];               // Provide LPPDMA description for ping-pong.

#define I2S0TX_PDMA_CH     (3)

S_BUFCTRL *psI2STX_BufCtrl = NULL;            // Provide Speaker to output data.
DSCT_T     sPDMA_I2STX[2];                    // Provide PDMA description for ping-pong.

// LPPMDA =========================================================================================================
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32TDStatus = LPPDMA_GET_TD_STS(LPPDMA);

    if (u32TDStatus & (1 << DMIC_LPPDMA_CH))
    {
        //    printf("LPPDMA_IRQHandler.DMIC0->STATUS %x\n",DMIC0->STATUS);
        LPPDMA_CLR_TD_FLAG(LPPDMA, (1 << DMIC_LPPDMA_CH));

        if (psDMIC_BufCtrl->u16DataCount <= (psDMIC_BufCtrl->u16BufCount / 2))
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

// PMDA =========================================================================================================
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32TDStatus = PDMA_GET_TD_STS(PDMA0);

    if (u32TDStatus & (1 << I2S0TX_PDMA_CH))
    {
        //    printf("PDMA0_IRQHandler.I2S0->STATUS1 %x\n",I2S0->STATUS1);
        PDMA_CLR_TD_FLAG(PDMA0, (1 << I2S0TX_PDMA_CH));

        if (psI2STX_BufCtrl->u16DataCount >= (psI2STX_BufCtrl->u16BufCount / 2))
            psI2STX_BufCtrl->u16DataCount -= (psI2STX_BufCtrl->u16BufCount / 2);

        if ((psI2STX_BufCtrl->u16ReadIdx += (psI2STX_BufCtrl->u16BufCount / 2)) >= psI2STX_BufCtrl->u16BufCount)
            psI2STX_BufCtrl->u16ReadIdx = 0;
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

// Microphone(DMIC)= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
void DMIC_Init(S_BUFCTRL *psInBufCtrl)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    // Select DMIC CLK source from PLL.
    CLK_SetModuleClock(DMIC0_MODULE, CLK_DMICSEL_DMIC0SEL_APLL1_DIV2, MODULE_NoMsk);
    // Enable DMIC clock.
    CLK_EnableModuleClock(DMIC0_MODULE);
    // DPWM IPReset.
    SYS_ResetModule(SYS_DMIC0RST);
    /* Lock protected registers */
    SYS_LockReg();
    // Set down sample rate 100 for quilty.(Suggest 96M used DMIC_CTL_DOWNSAMPLE_100_50 )
    DMIC_SET_DOWNSAMPLE(DMIC0, DMIC_DOWNSAMPLE_128);
    // Set DMIC sample rate.
    s_SampleRate = DMIC_SetSampleRate(DMIC0, SAMPLE_RATE);
    printf("DMIC SampleRate is %d\n", s_SampleRate);
    // Set channel's latch data falling type.
    //DMIC_SET_LATCHEDGE_CH01(DMIC0, DMIC_LATCHDATA_CH01FR);
    //DMIC_SET_LATCHEDGE_CH23(DMIC0, DMIC_LATCHDATA_CH23FR);
    // HPF control
    //DMIC_EnableHPF(DMIC0, DMIC_CTL_CH01HPFEN_Msk | DMIC_CTL_CH23HPFEN_Msk);
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
    //DMIC_SetDSPGainVolume(DMIC0, DMIC_CTL_CHEN0_Msk, 36);//+36dB

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
    //    SET_DMIC0_DAT_PE8();
    //    SET_DMIC0_CLK_PE9();
    //    SYS->GPE_MFOS = BIT8;
    //    PE8 = 1;
    SET_DMIC0_DAT_PB5();
    SET_DMIC0_CLK_PB4();
    SYS->GPB_MFOS = BIT5;
    PB5 = 1;
    // Config DMIC buffer control
    psDMIC_BufCtrl = psInBufCtrl;
}

void DMIC_Start(void)
{
    if (psDMIC_BufCtrl != NULL)
    {
        DMIC_ENABLE_CHANNEL(DMIC0, DMIC_CTL_CHEN0_Msk);
        DMIC_ENABLE_LPPDMA(DMIC0);
    }
}

void DMIC_Stop(void)
{
    DMIC_DISABLE_LPPDMA(DMIC0);
    DMIC_DISABLE_CHANNEL(DMIC0, DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk);
}

// Speaker(DPWM) = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
void I2STX_Init(S_BUFCTRL *psOutBufCtrl)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    // Select I2S CLK source from PCLK.
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_PCLK1, MODULE_NoMsk);
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
    // I2S0 Configuration
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    //Enable MCLK
    printf("I2S MCLK is %d\n", I2S_EnableMCLK(I2S0, 12000000));

    printf("  DMIC to I2S0 through FIFO test\n");

    // Clear TX FIFO buffer
    I2S_CLR_TX_FIFO(I2S0);

    // SPK(TX) buffer description
    sPDMA_I2STX[0].CTL = ((psOutBufCtrl->u16BufCount - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    sPDMA_I2STX[0].SA = (uint32_t) & (psOutBufCtrl->pai32Buf[0]);
    sPDMA_I2STX[0].DA = (uint32_t)(&I2S0->TXFIFO);
    sPDMA_I2STX[0].NEXT = (uint32_t)&sPDMA_I2STX[1];
    sPDMA_I2STX[1].CTL = ((psOutBufCtrl->u16BufCount - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    sPDMA_I2STX[1].SA = (uint32_t) & (psOutBufCtrl->pai32Buf[psOutBufCtrl->u16BufCount / 2]);
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

    // Config DPWM(Speaker) buffer control
    psI2STX_BufCtrl = psOutBufCtrl;
}

void I2STX_Start(void)
{
    if (psI2STX_BufCtrl != NULL)
    {
        I2S_ENABLE_TX(I2S0);
        I2S_ENABLE_TXDMA(I2S0);
    }
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

    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL1_SELECT);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
#if defined(ALIGN_AF_PINS)
    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);
    SYS_ResetModule(SYS_I2C3RST);
#else
    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);
    SYS_ResetModule(SYS_I2C2RST);
#endif
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);

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

#if defined(ALIGN_AF_PINS)
    CLK_EnableModuleClock(GPIOG_MODULE);
    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;
#else
    CLK_EnableModuleClock(GPIOD_MODULE);
    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
#endif
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
    int32_t i32Data[4];
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
#if defined(ALIGN_AF_PINS)
    SET_GPIO_PB12();
    GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);
    PB12 = 0;
#else
    SET_GPIO_PD4();
    GPIO_SetMode(PD, BIT4, GPIO_MODE_OUTPUT);
    PD4 = 0;
#endif

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

    // These defines are from  BUFCTRL.h for buffer control in this samle.
    // Buffer control handler configuration.
    BUFCTRL_CFG((&sInBufCtrl), ai32InBuf, sizeof(ai32InBuf) / sizeof(uint32_t));
    BUFCTRL_CFG((&sOutBufCtrl), ai32OutBuf, sizeof(ai32OutBuf) / sizeof(uint32_t));

    // Initiate microphone.
    DMIC_Init((S_BUFCTRL *)&sInBufCtrl);
    // Initiate speaker.
    I2STX_Init((S_BUFCTRL *)&sOutBufCtrl);
    // Start microphone.
    DMIC_Start();
    // Start speaker.
    I2STX_Start();

    // while loop for processing.
    while (1)
    {
        while (BUFCTRL_GET_COUNT((&sInBufCtrl)) >= 4 && !BUFCTRL_IS_FULL((&sOutBufCtrl)))
        {
            BUFCTRL_READ((&sInBufCtrl), &i32Data[0]);
            BUFCTRL_READ((&sInBufCtrl), &i32Data[1]);
            BUFCTRL_READ((&sInBufCtrl), &i32Data[2]);
            BUFCTRL_READ((&sInBufCtrl), &i32Data[3]);
            // 4 channel mixer to 2 channe
            //          i32Data[0] = i32Data[0]+i32Data[2];
            //          i32Data[1] = i32Data[1]+i32Data[3];
            BUFCTRL_WRITE((&sOutBufCtrl), i32Data[0]);
            BUFCTRL_WRITE((&sOutBufCtrl), i32Data[1]);
            BUFCTRL_WRITE((&sOutBufCtrl), i32Data[2]);
            BUFCTRL_WRITE((&sOutBufCtrl), i32Data[3]);
        }
    };
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
