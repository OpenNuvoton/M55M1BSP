/**************************************************************************//**
 * @file     img_sensor.c
 * @version  V1.00
 * @brief    image sensor driver
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "ImageSensor.h"
#include "Sensor.h"

typedef struct s_output_format
{
    uint32_t    m_u32OutputFormat;
    uint32_t    m_u32BytePerPixel;
    const char *m_strFormatName;
} S_OUTPUT_FORMAT;

const S_OUTPUT_FORMAT s_sOutputFormat[] =
{
    { CCAP_PKT_OUTFMT_YUV422,         2, "YUV422",         },
    { CCAP_PKT_OUTFMT_RGB555,         2, "RGB555",         },
    { CCAP_PKT_OUTFMT_RGB565,         2, "RGB565",         },
    { CCAP_PKT_OUTFMT_ONLY_Y,         1, "ONLY_Y",         },
    { CCAP_PKT_OUTFMT_ONLY_Y,         1, "1BIT",           },
    { CCAP_PKT_OUTFMT_RGB888_U8,      3, "RGB888_U8",      },
    { CCAP_PKT_OUTFMT_BGR888_U8,      3, "BGR888_U8",      },
    { CCAP_PKT_OUTFMT_RGB888_I8,      3, "RGB888_I8",      },
    { CCAP_PKT_OUTFMT_BGR888_I8,      3, "BGR888_I8",      },
    { CCAP_PKT_OUTFMT_ARGB888_U8,     4, "ARGB888_U8",     },
    { CCAP_PKT_OUTFMT_BGRA888_U8,     4, "BGRA888_U8",     },
    { CCAP_PKT_OUTFMT_ARGB888_I8,     4, "ARGB888_I8",     },
    { CCAP_PKT_OUTFMT_BGRA888_I8,     4, "BGRA888_I8",     },
};

/*------------------------------------------------------------------------------------------*/
/*  CCAP_IRQHandler                                                                          */
/*------------------------------------------------------------------------------------------*/
void CCAP_InterruptHandler(void)
{
    //g_u32FramePass++;
    //printf("Frame count: %d\n", g_u32FramePass);
}

void CCAP_IRQHandler(void)
{
    uint32_t u32CCAP_Status = CCAP->INTSTS;

    if ((CCAP->INTEN & CCAP_INTEN_VIEN_Msk) && (u32CCAP_Status & CCAP_INTSTS_VINTF_Msk))
    {
        CCAP_InterruptHandler();
        CCAP->INTSTS |= CCAP_INTSTS_VINTF_Msk;        /* Clear Frame end interrupt */
    }

    if ((CCAP->INTEN & CCAP_INTEN_ADDRMIEN_Msk) && (u32CCAP_Status & CCAP_INTSTS_ADDRMINTF_Msk))
    {
        CCAP->INTSTS |= CCAP_INTSTS_ADDRMINTF_Msk; /* Clear Address match interrupt */
    }

    if ((CCAP->INTEN & CCAP_INTEN_MEIEN_Msk) && (u32CCAP_Status & CCAP_INTSTS_MEINTF_Msk))
    {
        CCAP->INTSTS |= CCAP_INTSTS_MEINTF_Msk;    /* Clear Memory error interrupt */
    }

    CCAP->CTL = CCAP->CTL | CCAP_CTL_UPDATE;
    __DSB();
    __ISB();
}


void CCAP_SetFreq(uint32_t u32CCAP_ClkSrc, uint32_t u32SensorFreq)
{
    int32_t  i32Div;
    uint32_t u32CCAP_Clk;

    /* Unlock protected registers */
    SYS_UnlockReg();

    if (u32CCAP_ClkSrc == CLK_CCAPSEL_CCAP0SEL_HIRC)
        u32CCAP_Clk = __HIRC;
    else if (u32CCAP_ClkSrc == CLK_CCAPSEL_CCAP0SEL_MIRC)
        u32CCAP_Clk = CLK_GetMIRCFreq();
    else if (u32CCAP_ClkSrc == CLK_CCAPSEL_CCAP0SEL_HCLK2)
        u32CCAP_Clk = CLK_GetHCLK2Freq();
    else if (u32CCAP_ClkSrc == CLK_CCAPSEL_CCAP0SEL_APLL0_DIV2)
        u32CCAP_Clk = CLK_GetAPLL0ClockFreq() / 2;
    else if (u32CCAP_ClkSrc == CLK_CCAPSEL_CCAP0SEL_HXT)
        u32CCAP_Clk = CLK_GetHXTFreq();
    else
    {
        printf("Invalid CCAP clock source !\n");
        return ;
    }

    CLK_SetModuleClock(CCAP0_MODULE, u32CCAP_ClkSrc, 0);

    if (u32SensorFreq <= u32CCAP_Clk)
    {
        i32Div = (u32CCAP_Clk / u32SensorFreq) - 1;

        if (i32Div < 0)
            i32Div = 0;
    }
    else
    {
        i32Div = 0;
    }

    CLK->VSENSEDIV = (CLK->VSENSEDIV & ~CLK_VSENSEDIV_VSENSEDIV_Msk) | (i32Div << CLK_VSENSEDIV_VSENSEDIV_Pos);

    /* Lock protected registers */
    SYS_LockReg();

    printf("CCAP   engine clock: %d Hz\n", u32CCAP_Clk);
    printf("Target sensor clock: %d Hz.\n", u32SensorFreq);
    printf("Actual sensor clock: %d Hz. Divider=%d+1\n", u32CCAP_Clk / (i32Div + 1), i32Div);
}

static void MFP_ConfigCCAP(uint32_t bConfigCCAP)
{
    if (!bConfigCCAP)
    {
        // Config as GPIO

#if defined(M55M1_ETM_BOARD)
        // ETM Adapater Board Sensor 0
        SET_GPIO_PA1();
        SET_GPIO_PA0();
        SET_GPIO_PC5();
        SET_GPIO_PC4();
        SET_GPIO_PC3();
        SET_GPIO_PC2();
        SET_GPIO_PC1();
        SET_GPIO_PC0();

        SET_GPIO_PG9();
        SET_GPIO_PG10();
        SET_GPIO_PG12();
        SET_GPIO_PD7();
#else
        // NuMaker-M55M1 Sensor 0
        SET_GPIO_PG2();
        SET_GPIO_PG3();
        SET_GPIO_PG4();
        SET_GPIO_PF11();
        SET_GPIO_PF10();
        SET_GPIO_PF9();
        SET_GPIO_PF8();
        SET_GPIO_PF7();

        SET_GPIO_PG9();
        SET_GPIO_PG10();
        SET_GPIO_PG12();
        SET_GPIO_PD7();
#endif

    }
    else
    {

        /* Set multi-function pins for CCAP in DataFlow */

#if defined(M55M1_ETM_BOARD)
        // ETM Adapater Board Sensor 0
        SET_CCAP_DATA7_PA1();
        SET_CCAP_DATA6_PA0();
        SET_CCAP_DATA5_PC5();
        SET_CCAP_DATA4_PC4();
        SET_CCAP_DATA3_PC3();
        SET_CCAP_DATA2_PC2();
        SET_CCAP_DATA1_PC1();
        SET_CCAP_DATA0_PC0();

        SET_CCAP_PIXCLK_PG9();
        SET_CCAP_SCLK_PG10();
        SET_CCAP_VSYNC_PG12();
        SET_CCAP_HSYNC_PD7();

#else
        // NuMaker-M55M1 Sensor 0
        SET_CCAP_DATA7_PG2();
        SET_CCAP_DATA6_PG3();
        SET_CCAP_DATA5_PG4();
        SET_CCAP_DATA4_PF11();
        SET_CCAP_DATA3_PF10();
        SET_CCAP_DATA2_PF9();
        SET_CCAP_DATA1_PF8();
        SET_CCAP_DATA0_PF7();

        SET_CCAP_PIXCLK_PG9();
        SET_CCAP_SCLK_PG10();
        SET_CCAP_VSYNC_PG12();
        SET_CCAP_HSYNC_PD7();
#endif
    }
}

static S_SENSOR_INFO *s_psSensorInfo = NULL;

int ImageSensor_Init(void)
{
    /* Init Engine clock and Sensor clock */
    CCAP_SetFreq(CLK_CCAPSEL_CCAP0SEL_HCLK2, 48000000);
    MFP_ConfigCCAP(TRUE);

    /* Init sensor */
    //s_psSensorInfo = &g_sSensorHM1055_VGA_YUV422;
    s_psSensorInfo = &g_sSensorHM1055_QVGA_YUV422;

    /* Initialize sensor and set sensor output format as YUV422 */
    if (s_psSensorInfo->pfnInitSensor(0) == FALSE) return -1;

    return 0;
}

int ImageSensor_Config(E_IMAGE_FMT eImgFmt, uint32_t u32ImgWidth, uint32_t u32ImgHeight, bool bKeepRatio)
{
    uint32_t u32CropWinWidth;
    uint32_t u32CropWinHeight;
    uint32_t u32CropWinX = 0;
    uint32_t u32CropWinY = 0;

    if (bKeepRatio)
    {
        float fCorpFactoryW;
        float fCorpFactoryH;
        float fCorpFactory;
        fCorpFactoryW = (float)s_psSensorInfo->m_u16Width / u32ImgWidth;
        fCorpFactoryH = (float)s_psSensorInfo->m_u16Height / u32ImgHeight;

        if (fCorpFactoryW > fCorpFactoryH)
            fCorpFactory = fCorpFactoryH;
        else
            fCorpFactory = fCorpFactoryW;

        u32CropWinWidth = (uint32_t)(u32ImgWidth * fCorpFactory);
        u32CropWinHeight = (uint32_t)(u32ImgHeight * fCorpFactory);

        //centre
        u32CropWinX = (s_psSensorInfo->m_u16Width - u32CropWinWidth) / 2;
        u32CropWinY = (s_psSensorInfo->m_u16Height - u32CropWinHeight) / 2;

    }
    else
    {
        u32CropWinWidth = s_psSensorInfo->m_u16Width;
        u32CropWinHeight = s_psSensorInfo->m_u16Height;
    }


    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CCAP_SetCroppingWindow(u32CropWinY, u32CropWinX, u32CropWinHeight, u32CropWinWidth);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor and CCAP format and Order */
    CCAP_OpenPipes(
        s_psSensorInfo->m_u32Polarity,
        s_psSensorInfo->m_u32InputFormat,
        s_sOutputFormat[eImgFmt].m_u32OutputFormat,
        CCAP_PLN_DISABLED
    );

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CCAP_SetPacketScaling(u32ImgHeight, u32CropWinHeight, u32ImgWidth, u32CropWinWidth);
    printf("sensor input width %d \n", s_psSensorInfo->m_u16Width);
    printf("sensor input height %d \n", s_psSensorInfo->m_u16Height);
    printf("scaled image width %u \n", u32ImgWidth);
    printf("scaled image height %u \n", u32ImgHeight);

    /* Set Packet Frame Output Pixel Stride Width */
    CCAP_SetPacketStride(u32ImgWidth);

    /* Enable External CAP Interrupt */
    NVIC_EnableIRQ(CCAP_IRQn);

    return 0;
}


int ImageSensor_Capture(uint32_t u32FrameBufAddr)
{
    int i32Ret = CCAP_OK;
    /* Set System Memory Packet Base Address Register */
    //printf("sensor capture address %x \n", u32FrameBufAddr);
    CCAP_SetPacketBuf((uint32_t)u32FrameBufAddr);

    /* Start image capture */
    CCAP_Start();

    /* Start image capture */
    i32Ret = CCAP_Stop(TRUE);

    if (i32Ret != CCAP_OK)
        return -1;

    return 0;
}

int ImageSensor_TriggerCapture(uint32_t u32FrameBufAddr)
{
    /* Set System Memory Packet Base Address Register */
    //printf("sensor capture address %x \n", u32FrameBufAddr);
    CCAP_SetPacketBuf((uint32_t)u32FrameBufAddr);

    /* Trigger image capture */
    CCAP_Start();
    CCAP->CTL |= CCAP_CTL_SHUTTER_Msk;

    return 0;
}

int ImageSensor_WaitCaptureDone(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock << 1; /* 2 second */

    while (!CCAP_IS_STOPPED())
    {
        if (--u32TimeOutCnt == 0) return CCAP_ERR_TIMEOUT;
    }

    return 0;
}
