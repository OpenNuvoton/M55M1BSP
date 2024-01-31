/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   This is an I2S demo with PDMA function connected with codec.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"

//------------------------------------------------------------------------------
#define NAU8822             1
#if defined(ALIGN_AF_PINS)
    #define I2C_PORT                        I2C3
#else
    #define I2C_PORT                        I2C2
#endif

//------------------------------------------------------------------------------
void SYS_Init(void);
void PDMA_Init(void);
void I2C_Init(void);

#if NAU8822
    void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data);
    void NAU8822_Setup(void);
#else
    uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len);
    uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat);
    void NAU88L25_Reset(void);
    void NAU88L25_Setup(void);
#endif

//------------------------------------------------------------------------------
static uint32_t s_au32PcmRxBuff[2][BUFF_LEN] = {{0}};
static uint32_t s_au32PcmTxBuff[2][BUFF_LEN] = {{0}};
static DMA_DESC_T DMA_TXDESC[2], DMA_RXDESC[2];

static volatile uint8_t s_u8TxIdx = 0, s_u8RxIdx = 0;
static volatile uint8_t s_u8CopyData = 0;

//------------------------------------------------------------------------------
//NVT_ITCM
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & 0x2)
    {
        if (PDMA_GET_TD_STS(PDMA0) & 0x4)             /* channel 2 done */
        {
            /* Copy RX data to TX buffer */
            s_u8CopyData = 1;
            s_u8RxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }

        if (PDMA_GET_TD_STS(PDMA0) & 0x2)             /* channel 1 done */
        {
            s_u8TxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);
        }
    }

    //else
    //{
    //    printf("unknown interrupt, status=0x%x!!\n", u32Status);
    //}
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

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

#ifdef INPUT_IS_LIN   /* Input source is LIN */
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
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
#else   /* Input source is MIC */
    I2C_WriteNAU8822(1,  0x03F);
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

    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

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
    I2C_WriteNAU88L25(0x0033,  0x00CF);
    I2C_WriteNAU88L25(0x0034,  0x02CF);
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

void SYS_Init(void)
{
    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

#if defined(ALIGN_AF_PINS)
    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);
#else
    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);
#endif

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

#if defined(ALIGN_AF_PINS)
    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;
#else
    /* Set I2C3 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
#endif
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    /* Tx description */
    DMA_TXDESC[0].ctl = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_TXDESC[0].src = (uint32_t)&s_au32PcmTxBuff[0];
    DMA_TXDESC[0].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_TXDESC[0].offset = (uint32_t)&DMA_TXDESC[1];

    DMA_TXDESC[1].ctl = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_TXDESC[1].src = (uint32_t)&s_au32PcmTxBuff[1];
    DMA_TXDESC[1].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_TXDESC[1].offset = (uint32_t)&DMA_TXDESC[0];   /* Link to first description */

    /* Rx description */
    DMA_RXDESC[0].ctl = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_RXDESC[0].src = (uint32_t)&I2S0->RXFIFO;
    DMA_RXDESC[0].dest = (uint32_t)&s_au32PcmRxBuff[0];
    DMA_RXDESC[0].offset = (uint32_t)&DMA_RXDESC[1];

    DMA_RXDESC[1].ctl = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_RXDESC[1].src = (uint32_t)&I2S0->RXFIFO;
    DMA_RXDESC[1].dest = (uint32_t)&s_au32PcmRxBuff[1];
    DMA_RXDESC[1].offset = (uint32_t)&DMA_RXDESC[0];   /* Link to first description */

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(PDMA0, 0x3 << 1);

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA0, 1, PDMA_I2S0_TX, 1, (uint32_t)&DMA_TXDESC[0]);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_I2S0_RX, 1, (uint32_t)&DMA_RXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA0_IRQn);
}

/* Init I2C interface */
void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                I2S Driver Sample Code with audio codec                |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with audio codec.\n");

    /* Init I2C to access codec */
    I2C_Init();

#if (!NAU8822)
    /* Reset NAU88L25 codec */
    NAU88L25_Reset();
#endif

#ifdef INPUT_IS_LIN
    /* Open I2S0 interface and set to slave mode, stereo channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);
#else
    /* Open I2S0 interface and set to slave mode, mono channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S);
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

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, 0);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

#ifndef INPUT_IS_LIN
    /* NAU8822 will store data in left channel */
    I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_LEFT);
#endif

#if NAU8822
    /* Initialize NAU8822 codec */
    NAU8822_Setup();
#else
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    PDMA_Init();

    /* Enable I2S Rx function */
    I2S_ENABLE_RXDMA(I2S0);
    I2S_ENABLE_RX(I2S0);

    /* Enable I2S Tx function */
    I2S_ENABLE_TXDMA(I2S0);
    I2S_ENABLE_TX(I2S0);

    while (1)
    {
        if (s_u8CopyData)
        {
            memcpy(&s_au32PcmTxBuff[s_u8TxIdx ^ 1], &s_au32PcmRxBuff[s_u8RxIdx], BUFF_LEN * 4);
        }
    }
}
