/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    MP3 recorder sample encodes sound to MP3 format and stores it to
 *           a microSD card or a USB storage device, and this MP3 file can also be played.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "l3.h"
#include "hyperram_code.h"
#include "usbh_lib.h"

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
#ifdef __ICCARM__
    #pragma data_alignment=32

    #if (NVT_DCACHE_ON == 1)
        NVT_DTCM __ALIGNED(32) DMA_DESC_T DMA_DESC[2] @0x20003000;
    #else
        DMA_DESC_T DMA_DESC[2] @0x20003000;
    #endif
#else
    #if (NVT_DCACHE_ON == 1)
        NVT_DTCM __ALIGNED(32) DMA_DESC_T DMA_DESC[2];
    #else
        DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
    #endif
#endif

volatile uint32_t u32BTN0 = 0xF, u32BTN1 = 0xF;
volatile uint32_t g_u32RecordStart = 0, g_u32RecordDone = 0;
volatile uint32_t g_u32WriteSDToggle = 0;

extern shine_config_t config;
extern shine_t s;
extern int32_t samples_per_pass;
extern FIL mp3FileObject;
extern void MP3_Storage_Init(TCHAR *path);

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

DWORD get_fattime(void)
{
    DWORD tmr;

    tmr = 0x00000;

    return tmr;
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0/1 200MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL1_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

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
#if (USE_I2S == 1)
    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, 0);

    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);
#else
    /* Enable SPI peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select HIRC as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_HIRC, MODULE_NoMsk);

    /* Enable I2C clock */
    CLK_EnableModuleClock(I2C1_MODULE);
#endif

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();
#if (USE_I2S == 1)
    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;
#else
    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SET_SPI0_SS_PA3();
    SET_SPI0_CLK_PA2();
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();

    /* Enable SPII2S0 clock pin (PA.2) schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk);

    /* PA.4 is SPI0_I2SMCLK */
    SET_SPI0_I2SMCLK_PA4();

    /* Set PB multi-function pins for I2C1 */
    SYS->GPB_MFP0 = SYS_GPB_MFP0_PB1MFP_I2C1_SCL | SYS_GPB_MFP0_PB0MFP_I2C1_SDA;

    /* Enable I2C clock pinschmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
#endif

#ifndef REC_IN_RT
    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);

    /* Enable OTFC module clock */
    CLK_EnableModuleClock(OTFC0_MODULE);

    /* Init SPIM multi-function pins */
    SET_SPIM0_CLKN_PH12();
    SET_SPIM0_CLK_PH13();
    SET_SPIM0_D2_PJ5();
    SET_SPIM0_D3_PJ6();
    SET_SPIM0_D4_PH14();
    SET_SPIM0_D5_PH15();
    SET_SPIM0_D6_PG13();
    SET_SPIM0_D7_PG14();
    SET_SPIM0_MISO_PJ4();
    SET_SPIM0_MOSI_PJ3();
    SET_SPIM0_RESETN_PJ2();
    SET_SPIM0_RWDS_PG15();
    SET_SPIM0_SS_PJ7();

    PG->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk |
                  GPIO_SMTEN_SMTEN14_Msk |
                  GPIO_SMTEN_SMTEN15_Msk);
    PH->SMTEN |= (GPIO_SMTEN_SMTEN12_Msk |
                  GPIO_SMTEN_SMTEN13_Msk |
                  GPIO_SMTEN_SMTEN14_Msk |
                  GPIO_SMTEN_SMTEN15_Msk);
    PJ->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk |
                  GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk |
                  GPIO_SMTEN_SMTEN5_Msk |
                  GPIO_SMTEN_SMTEN6_Msk |
                  GPIO_SMTEN_SMTEN7_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PG, BIT13, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PG, BIT14, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PG, BIT15, GPIO_SLEWCTL_HIGH);

    GPIO_SetSlewCtl(PH, BIT12, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PH, BIT13, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PH, BIT14, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PH, BIT15, GPIO_SLEWCTL_HIGH);

    GPIO_SetSlewCtl(PJ, BIT2, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, BIT3, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, BIT4, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, BIT5, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, BIT6, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, BIT7, GPIO_SLEWCTL_HIGH);
#endif
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t u8Id)
{
    DMA_DESC[u8Id].ctl = (DMA_DESC[u8Id].ctl & ~PDMA_DSCT_CTL_TXCNT_Msk) |
                         ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
                         PDMA_OP_SCATTER;

#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr((void *)&DMA_DESC[u8Id], sizeof(DMA_DESC[u8Id]));

    if (u8Id == 0)
        SCB_CleanDCache_by_Addr((void *)g_au32PcmBuff1, PCM_BUFFER_SIZE * sizeof(uint32_t));
    else
        SCB_CleanDCache_by_Addr((void *)g_au32PcmBuff2, PCM_BUFFER_SIZE * sizeof(uint32_t));

#endif
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    uint32_t u32Peripheral = PDMA_I2S0_TX;

#if (USE_I2S == 1)
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&g_ai32PCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1];

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&g_ai32PCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0];

    u32Peripheral = PDMA_I2S0_TX;
#else
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&g_ai32PCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&SPI_PORT->TX;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1];

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&g_ai32PCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&SPI_PORT->TX;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0];

    u32Peripheral = PDMA_SPI0_TX;
#endif

    PDMA_Open(PDMA_PORT, (1 << I2S_TX_DMA_CH));
    PDMA_SetTransferMode(PDMA_PORT, I2S_TX_DMA_CH, u32Peripheral, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA_PORT, I2S_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQ);
}

void GPH_IRQHandler(void)
{
    volatile uint32_t u32Temp;

    /* To check if PH.1 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PH, BIT1))
    {
        GPIO_CLR_INT_FLAG(PH, BIT1);

        if (g_u32RecordStart == 1)
            g_u32RecordDone = 1;
    }
    else
    {
        /* Un-expected interrupt. Just clear all PH interrupts */
        u32Temp = PH->INTSRC;
        PH->INTSRC = u32Temp;
        printf("Un-expected interrupts.\n");
    }
}

#ifndef REC_IN_RT

static int32_t Clear4Bytes(uint32_t u32StartAddr)
{
    outp32(u32StartAddr, 0);
    return 0;
}

static int32_t ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for (i = u32StartAddr; i < u32EndAddr; i += 4)
    {
        if (Clear4Bytes(i) < 0)
        {
            return -1;
        }

        u32Data = inp32(i);

        if (u32Data != 0)
        {
            printf("ClearHyperRAM fail!! Read address:0x%08x  data::0x%08x  expect: 0\n",  i, u32Data);
            return -1;
        }
    }

    return 0;
}

#endif

int32_t main(void)
{
#ifdef REC_IN_RT
    int32_t i32Written;
    uint8_t *pu8Data;
    uint32_t u32PrintFlag = 1;
#endif

    TCHAR chStoragePath[] = { '0', ':', 0 };    /* SD drive started from 0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init I2C to access audio codec */
    I2C_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

#ifndef REC_IN_RT
    /* Init HyperRAM and Entry DMM Mode */
    HyperRAM_Init(SPIM_PORT);
#endif

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                  MP3 Recorder Sample with Audio Codec                 |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf(" Please insert a microSD card\n");
    printf(" Press BTN0 to start recording, and press BTN1 to stop.\n");
    printf(" Press BTN1 to play an MP3 file from storage.\n");

    MP3_Storage_Init(chStoragePath);

    /* Configure PI.11 and PH.1 as Output mode */
    GPIO_SetMode(PI, BIT11, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);

    /* Enable PH.1 interrupt by falling edge trigger */
    GPIO_EnableInt(PH, 1, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPH_IRQn);
    NVIC_SetPriority(GPH_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    while (1)
    {
        /* Read pin state of PH.0 and PH.1 */
        u32BTN0 = (PI->PIN & (1 << 11)) ? 0 : 1;
        u32BTN1 = (PH->PIN & (1 << 1)) ? 0 : 1;

#if (USE_SDH == 1)

        if (SD0.IsCardInsert == TRUE)
#else
        usbh_pooling_hubs();

#endif
        {
#ifdef REC_IN_RT

            /* Inform users about microSD card usage */
            if ((u32PrintFlag == 1) && (g_u32ErrorFlag != 0))
            {
                printf("\n\nSome sounds have been lost due to poor microSD card performance.\nPlease replace !!!\n\n");

                u32PrintFlag = 0;
                g_u32ErrorFlag = 0;
            }

            /* Encode sound data to MP3 format and store in microSD card */
            if (g_u32RecordStart == 1)
            {
                if ((g_u32WriteSDToggle == 0) && (g_u32BuffPos1 > 0) && (g_u32BuffPos1 == (uint32_t)((samples_per_pass * config.wave.channels) >> 1)))
                {
                    pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(&g_au32PcmBuff1), (int *)&i32Written);

                    if (Write_MP3(i32Written, pu8Data) != i32Written)
                    {
                        printf("shineenc: write error\n");
                    }

                    g_u32BuffPos1 = 0;
                    g_u32WriteSDToggle = 1;
                }
                else if ((g_u32WriteSDToggle == 1) && (g_u32BuffPos2 > 0) && (g_u32BuffPos2 == (uint32_t)((samples_per_pass * config.wave.channels) >> 1)))
                {
                    pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(&g_au32PcmBuff2), (int *)&i32Written);

                    if (Write_MP3(i32Written, pu8Data) != i32Written)
                    {
                        printf("shineenc: write error\n");
                    }

                    g_u32BuffPos2 = 0;
                    g_u32WriteSDToggle = 0;
                }
            }

#endif

            if ((u32BTN0 == 1) && (g_u32RecordStart == 0) && (g_u32RecordDone == 0))
            {
#ifndef REC_IN_RT

                /* Clear HyperRAM */
                if (ClearHyperRAM(SPIM_HYPER_DMM0_ADDR, SPIM_HYPER_DMM0_ADDR + 0x800000) < 0)
                    return -1;

#endif

                /* Configure recording condition, init I2S, audio codec and encoder */
                Recorder_Init();

#ifdef REC_IN_RT

                printf("Start recording ... (online)\n");

#else

                printf("Start recording ... (offline)\n");

#endif

#if (USE_I2S == 1)
                /* Enable I2S RX function to receive sound data */
                I2S_ENABLE_RX(I2S_PORT);
#else
                SPII2S_ENABLE_RX(SPI_PORT);
#endif
                g_u32RecordStart = 1;

#ifdef REC_IN_RT

                u32PrintFlag = 1;

                printf("Encode and write out the MP3 file ");

#endif
            }

            /* Play MP3 */
            if ((u32BTN1 == 1) && (g_u32RecordStart == 0))
            {
                MP3Player();
            }

            if (g_u32RecordDone == 1)
            {
#if (USE_I2S == 1)
                /* Disable I2S RX function */
                I2S_DISABLE_RX(I2S_PORT);
#else
                SPII2S_DISABLE_RX(SPI_PORT);
#endif

#ifndef REC_IN_RT

                /* Encode sound data to MP3 format and store in microSD card */
                MP3Recorder();

#endif

                /* Close encoder */
                shine_close(s);

                f_close(&mp3FileObject);

                printf(" Done !\n\n");

                g_u32RecordStart = 0;
                g_u32RecordDone = 0;

                /* Play MP3 */
                MP3Player();
            }
        }
    }
}
