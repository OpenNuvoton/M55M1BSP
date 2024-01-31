/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    MP3 recorder sample encodes sound to MP3 format and stores it to
 *           a microSD card, and this MP3 file can also be played.
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

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
#ifdef __ICCARM__
    #pragma data_alignment=32
    DMA_DESC_T DMA_DESC[2] @0x20003000;
#else
    DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
#endif

volatile uint32_t u32BTN0 = 0xF, u32BTN1 = 0xF;
volatile uint32_t g_u32RecordStart = 0, g_u32RecordDone = 0;
volatile uint32_t g_u32WriteSDToggle = 0;

extern shine_config_t config;
extern shine_t        s;
extern int32_t        samples_per_pass;
extern FIL            mp3FileObject;

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

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
}

void SDH0_IRQHandler(void)
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
            memset(&SD0, 0, sizeof(SDH_INFO_T));
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
}

void SD_Init(void)
{
    /* Select multi-function pins */
    SET_SD0_DAT0_PE2();
    SET_SD0_DAT1_PE3();
    SET_SD0_DAT2_PE4();
    SET_SD0_DAT3_PE5();
    SET_SD0_CLK_PE6();
    SET_SD0_CMD_PE7();
    SET_SD0_nCD_PD13();

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_SDHSEL_SDH0SEL_APLL1_DIV2, CLK_SDHDIV_SDH0DIV(2));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);

    NVIC_SetPriority(SDH0_IRQn, 3);
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0/1 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL1_SELECT);

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

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

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

#ifndef REC_IN_RT

    if (SPIM_PORT == SPIM0)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM0_CLKN_PC5();
        SET_SPIM0_CLK_PC4();
        SET_SPIM0_D2_PC0();
        SET_SPIM0_D3_PG10();
        SET_SPIM0_D4_PG9();
        SET_SPIM0_D5_PG13();
        SET_SPIM0_D6_PG14();
        SET_SPIM0_D7_PG15();
        SET_SPIM0_MISO_PG12();
        SET_SPIM0_MOSI_PG11();
        SET_SPIM0_RESETN_PC2();
        SET_SPIM0_RWDS_PC1();
        SET_SPIM0_SS_PC3();

        PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk |
                      GPIO_SMTEN_SMTEN1_Msk |
                      GPIO_SMTEN_SMTEN2_Msk |
                      GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk |
                      GPIO_SMTEN_SMTEN5_Msk);
        PG->SMTEN |= (GPIO_SMTEN_SMTEN9_Msk  |
                      GPIO_SMTEN_SMTEN10_Msk |
                      GPIO_SMTEN_SMTEN11_Msk |
                      GPIO_SMTEN_SMTEN12_Msk |
                      GPIO_SMTEN_SMTEN13_Msk |
                      GPIO_SMTEN_SMTEN14_Msk |
                      GPIO_SMTEN_SMTEN15_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PC, BIT0, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT1, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT2, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT5, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PG, BIT9, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT10, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT11, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT12, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT13, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT14, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT15, GPIO_SLEWCTL_HIGH);
    }
    else if (SPIM_PORT == SPIM1)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM1_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM1_CLKN_PH12();
        SET_SPIM1_CLK_PH13();
        SET_SPIM1_D2_PJ4();
        SET_SPIM1_D3_PJ3();
        SET_SPIM1_D4_PH15();
        SET_SPIM1_D5_PD7();
        SET_SPIM1_D6_PD6();
        SET_SPIM1_D7_PD5();
        SET_SPIM1_MISO_PJ5();
        SET_SPIM1_MOSI_PJ6();
        SET_SPIM1_RESETN_PJ2();
        SET_SPIM1_RWDS_PH14();
        SET_SPIM1_SS_PJ7();

        PD->SMTEN |= (GPIO_SMTEN_SMTEN5_Msk |
                      GPIO_SMTEN_SMTEN6_Msk |
                      GPIO_SMTEN_SMTEN7_Msk);
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
        GPIO_SetSlewCtl(PD, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PD, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PD, BIT7, GPIO_SLEWCTL_HIGH);

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
    }

#endif
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&g_ai32PCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1];

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&g_ai32PCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0];

    PDMA_Open(PDMA0, 1 << 2);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_I2S0_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA0, 2, 0);
    NVIC_EnableIRQ(PDMA0_IRQn);
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
    int32_t i32Written;
    uint8_t *pu8Data;
    uint32_t u32PrintFlag = 1;

    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Init SD */
    SD_Init();

#ifndef REC_IN_RT
    /* Init HyperRAM and Entry DMM Mode */
    HyperRAM_Init(SPIM_PORT);
#endif

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                  MP3 Recorder Sample with Audio Codec                 |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf(" Please insert a microSD card\n");
    printf(" Press BTN0 button to start recording and press BTN1 button to stop recording\n");
    printf(" Press BTN1 button can also play MP3 file from microSD card\n");

    /* Configure PI.11 and PH.1 as Output mode */
    GPIO_SetMode(PI, BIT11, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);

    /* Enable PH.1 interrupt by falling edge trigger */
    GPIO_EnableInt(PH, 1, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPH_IRQn);
    NVIC_SetPriority(GPH_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    /* Configure FATFS */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* Set default path */

    /* Init I2C to access audio codec */
    I2C_Init();

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, 0);

    while (1)
    {
        /* Read pin state of PH.0 and PH.1 */
        u32BTN0 = (PI->PIN & (1 << 11)) ? 0 : 1;
        u32BTN1 = (PH->PIN & (1 << 1)) ? 0 : 1;

        if (SD0.IsCardInsert == TRUE)
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
                if ((g_u32WriteSDToggle == 0) && (g_u32BuffPos1 > 0) && (g_u32BuffPos1 == ((samples_per_pass * config.wave.channels) >> 1)))
                {
                    pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(&g_au32PcmBuff1), (int *)&i32Written);

                    if (Write_MP3(i32Written, pu8Data, &config) != i32Written)
                    {
                        printf("shineenc: write error\n");
                    }

                    g_u32BuffPos1 = 0;
                    g_u32WriteSDToggle = 1;
                }
                else if ((g_u32WriteSDToggle == 1) && (g_u32BuffPos2 > 0) && (g_u32BuffPos2 == ((samples_per_pass * config.wave.channels) >> 1)))
                {
                    pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(&g_au32PcmBuff2), (int *)&i32Written);

                    if (Write_MP3(i32Written, pu8Data, &config) != i32Written)
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
                if (ClearHyperRAM(HYPER_RAM_MEM_MAP, HYPER_RAM_MEM_MAP + 0x800000) < 0)
                    return -1;

#endif

                /* Configure recording condition, init I2S, audio codec and encoder */
                Recorder_Init();

#ifdef REC_IN_RT

                printf("Start recording ... (online)\n");

#else

                printf("Start recording ... (offline)\n");

#endif

                /* Enable I2S RX function to receive sound data */
                I2S_ENABLE_RX(I2S0);

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
                /* Disable I2S RX function */
                I2S_DISABLE_RX(I2S0);

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
