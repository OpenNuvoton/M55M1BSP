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
    { CCAP_PAR_OUTFMT_YUV422,         2, "YUV422",         },
    { CCAP_PAR_OUTFMT_RGB555,         2, "RGB555",         },
    { CCAP_PAR_OUTFMT_RGB565,         2, "RGB565",         },
    { CCAP_PAR_OUTFMT_ONLY_Y,         1, "ONLY_Y",         },
    { CCAP_PAR_OUTFMT_ONLY_Y,         1, "1BIT",           },
    { CCAP_PAR_OUTFMT_RGB888_U8,      3, "RGB888_U8",      },
    { CCAP_PAR_OUTFMT_BGR888_U8,      3, "BGR888_U8",      },
    { CCAP_PAR_OUTFMT_RGB888_I8,      3, "RGB888_I8",      },
    { CCAP_PAR_OUTFMT_BGR888_I8,      3, "BGR888_I8",      },
    { CCAP_PAR_OUTFMT_ARGB888_U8,     4, "ARGB888_U8",     },
    { CCAP_PAR_OUTFMT_BGRA888_U8,     4, "BGRA888_U8",     },
    { CCAP_PAR_OUTFMT_ARGB888_I8,     4, "ARGB888_I8",     },
    { CCAP_PAR_OUTFMT_BGRA888_I8,     4, "BGRA888_I8",     },
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

#define CLK_CCAPSEL_CCAP0SEL_HCLK2           (0x1UL << CLK_CCAPSEL_CCAP0SEL_Pos)         /*!< Select CCAP sensor clock source from HIRC \hideinitializer */


// u32ModFreqKHz is fixed
static int CCAP_SetFreq(uint32_t u32ModFreqKHz, uint32_t u32SensorFreq)
{
    int32_t i32Div;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable CCAP Clock */
    CLK_EnableModuleClock(CCAP0_MODULE);

    /* Reset IP */
    SYS_ResetModule(SYS_CCAP0RST);

    if (u32ModFreqKHz < __HIRC)
    {
        CLK_SetModuleClock(CCAP0_MODULE, CLK_CCAPSEL_CCAP0SEL_MIRC, 0);
        i32Div = (CLK_GetMIRCFreq() / u32SensorFreq) - 1;
    }
    else
    {
        CLK_SetModuleClock(CCAP0_MODULE, CLK_CCAPSEL_CCAP0SEL_HXT, 0);
        i32Div = (CLK_GetHCLK2Freq() / u32SensorFreq) - 1;
        //CLK_SetModuleClock(CCAP0_MODULE, CLK_CCAPSEL_CCAP0SEL_HIRC, 0);
        //i32Div = (__HIRC / u32SensorFreq) - 1;
    }

    if (i32Div < 0)
        i32Div = 0;

    CLK->VSENSEDIV = (CLK->VSENSEDIV & ~CLK_VSENSEDIV_VSENSEDIV_Msk) | (i32Div << CLK_VSENSEDIV_VSENSEDIV_Pos);

    /* lock protected registers */
    SYS_LockReg();

    return 0;
}

static void MFP_ConfigCCAP(uint32_t bConfigCCAP)
{
    if (!bConfigCCAP)
    {
        // Config as GPIO
        SET_GPIO_PG2();
        SET_GPIO_PG3();
        SET_GPIO_PG4();
        SET_GPIO_PF11();
        SET_GPIO_PF10();
        SET_GPIO_PF9();
        SET_GPIO_PF8();
        SET_GPIO_PF7();

#if 1
        // NuMaker-M55M1 Sensor 0
        SET_GPIO_PG9();
        SET_GPIO_PG10();
        SET_GPIO_PG12();
        SET_GPIO_PG13();
#else
        // ETM Adapater Board Sensor 0
        SET_GPIO_PB13();
        SET_GPIO_PB12();
        SET_GPIO_PB10();
        SET_GPIO_PB9();
#endif
    }
    else
    {

        /* Set multi-function pins for CCAP in DataFlow */
        SET_CCAP_DATA7_PG2();
        SET_CCAP_DATA6_PG3();
        SET_CCAP_DATA5_PG4();
        SET_CCAP_DATA4_PF11();
        SET_CCAP_DATA3_PF10();
        SET_CCAP_DATA2_PF9();
        SET_CCAP_DATA1_PF8();
        SET_CCAP_DATA0_PF7();

#if 1
        // NuMaker-M55M1 Sensor 0
        SET_CCAP_PIXCLK_PG9();
        SET_CCAP_SCLK_PG10();
        SET_CCAP_VSYNC_PG12();
        SET_CCAP_HSYNC_PG13();
#else
        // ETM Adapater Board Sensor 0
        SET_CCAP_PIXCLK_PB13();
        SET_CCAP_SCLK_PB12();
        SET_CCAP_VSYNC_PB10();
        SET_CCAP_HSYNC_PB9();
#endif
    }
}

static S_SENSOR_INFO *s_psSensorInfo = NULL;

int ImageSensor_Init(void)
{
    /* Init Engine clock and Sensor clock */
    CCAP_SetFreq(24000000, 24000000);
    MFP_ConfigCCAP(TRUE);

    /* Init sensor */
    //s_psSensorInfo = &g_sSensorRGB565_640x480;    //rgb565 vga sensor
    s_psSensorInfo = &g_sSensorHM1055_VGA_YUV422;
    //i32RetCode = s_psSensorInfo->m_pfnInitFunc(8);

    /* Initialize sensor and set sensor output format as YUV422 */
    if (s_psSensorInfo->pfnInitSensor(0) == FALSE) return -1;

    return 0;
}

int ImageSensor_Config(E_IMAGE_FMT eImgFmt, uint32_t u32ImgWidth, uint32_t u32ImgHeight)
{
    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CCAP_SetCroppingWindow(0, 0, s_psSensorInfo->m_u16Height, s_psSensorInfo->m_u16Width);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor and CCAP format and Order */
    CCAP_Open(
        s_psSensorInfo->m_u32Polarity,
        s_psSensorInfo->m_u32InputFormat,
        s_sOutputFormat[eImgFmt].m_u32OutputFormat
    );

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CCAP_SetPacketScaling(u32ImgHeight, s_psSensorInfo->m_u16Height, u32ImgWidth, s_psSensorInfo->m_u16Width);
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
