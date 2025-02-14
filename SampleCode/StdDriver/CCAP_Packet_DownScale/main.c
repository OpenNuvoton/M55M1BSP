/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use packet format (all the luma and chroma data interleaved) to
 *           store captured image from sensor to SRAM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sensor.h"

#define CCAP_OUTPUT_WIDTH       320
#define CCAP_OUTPUT_HEIGHT      240
NVT_NOINIT uint8_t g_au8FrameBuffer[DCACHE_ALIGN_LINE_SIZE(CCAP_OUTPUT_WIDTH * CCAP_OUTPUT_HEIGHT * 2)] __ALIGNED(DCACHE_LINE_SIZE);

/*------------------------------------------------------------------------------------------*/
/* To run CCAPInterruptHandler, when CCAP frame end interrupt                               */
/*------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32FramePass = 0;

/*------------------------------------------------------------------------------------------*/
/*  CCAP_IRQHandler                                                                         */
/*------------------------------------------------------------------------------------------*/
NVT_ITCM void CCAP_IRQHandler(void)
{
    int32_t  i32TimeoutCnt;
    uint32_t u32IntStatus = CCAP_GET_INT_STS();

    if (CCAP_IsIntEnabled(CCAP_INT_VIEN_ENABLE) && (u32IntStatus & CCAP_INTSTS_VINTF_Msk) == CCAP_INTSTS_VINTF_Msk)
    {
#if (NVT_DCACHE_ON == 1)
        /* Invalidate the data cache for the received frame buffer to ensure data coherency if D-Cache is enabled. */
        SCB_InvalidateDCache_by_Addr(g_au8FrameBuffer, sizeof(g_au8FrameBuffer));
#endif  // (NVT_DCACHE_ON == 1)
        g_u32FramePass++;
        CCAP_CLR_INT_FLAG(CCAP_INTSTS_VINTF_Msk);   /* Clear Frame end interrupt */
    }

    if (CCAP_IsIntEnabled(CCAP_INT_ADDRMIEN_ENABLE) && (u32IntStatus & CCAP_INTSTS_ADDRMINTF_Msk) == CCAP_INTSTS_ADDRMINTF_Msk)
    {
        CCAP_CLR_INT_FLAG(CCAP_INTSTS_ADDRMINTF_Msk);   /* Clear Address match interrupt */
    }

    if (CCAP_IsIntEnabled(CCAP_INT_MEIEN_ENABLE) && (u32IntStatus & CCAP_INTSTS_MEINTF_Msk) == CCAP_INTSTS_MEINTF_Msk)
    {
        CCAP_CLR_INT_FLAG(CCAP_INTSTS_MEINTF_Msk);     /* Clear Memory error interrupt */
    }

    CCAP_EnableUpdate();

    // Loop until either the flag is cleared or timeout occurs
    i32TimeoutCnt = 1000;

    while (1)
    {
        // Check flag is cleared
        if (CCAP_GET_INT_STS() == 0)
        {
            break;
        }

        // Check timeout
        if (i32TimeoutCnt-- == 0)
        {
            printf("Clear interrupt flag timeout !\n");
            break;
        }
    }
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

int32_t PacketFormatDownScale(S_SENSOR_INFO *psSensorInfo)
{
    uint32_t u32Frame;

    /* Initialize sensor and set output format as YUV422 */
    if (psSensorInfo->pfnInitSensor((uint32_t)psSensorInfo) == FALSE)
    {
        printf("Init sensor failed !\n");
        return -1;
    }

    /* Enable External CCAP Interrupt */
    NVIC_EnableIRQ(CCAP_IRQn);

    /* Enable CCAP Frame End Interrupt */
    CCAP_EnableInt(CCAP_INT_VIEN_ENABLE);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor Format and Order */
    CCAP_Open((psSensorInfo->m_u32Polarity | psSensorInfo->m_u32InputFormat | CCAP_PAR_OUTFMT_YUV422), CCAP_CTL_PKTEN);

    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CCAP_SetCroppingWindow(0, 0, psSensorInfo->m_u16Height, psSensorInfo->m_u16Width);

    /* Set System Memory Packet Base Address Register */
    CCAP_SetPacketBuf((uint32_t)g_au8FrameBuffer);

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CCAP_SetPacketScaling(CCAP_OUTPUT_HEIGHT, psSensorInfo->m_u16Height, CCAP_OUTPUT_WIDTH, psSensorInfo->m_u16Width);

    /* Set Packet Frame Output Pixel Stride Width */
    CCAP_SetPacketStride(CCAP_OUTPUT_WIDTH);

    /* Start Image Capture Interface */
    CCAP_Start();

    g_u32FramePass = 0;
    u32Frame = g_u32FramePass;

    while (1)
    {
        if (u32Frame != g_u32FramePass)
        {
            u32Frame = g_u32FramePass;
            printf("Get frame %d\n", u32Frame);
        }
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(CCAP0_MODULE);
    /* Reset IP */
    SYS_ResetModule(SYS_CCAP0RST);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set multi-function pins for CCAP */
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

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+--------------------------------------------+\n");
    printf("|  M55M1 CCAP Packet Down Scale Sample Code  |\n");
    printf("+--------------------------------------------+\n");

    /* Init CCAP clock and Sensor clock */
    CCAP_SetFreq(CLK_CCAPSEL_CCAP0SEL_HCLK2, 24000000);

    /* Using Packet format to Image down scale */
    if (PacketFormatDownScale(&g_sSensorHM1055) != 0)
        printf("Capture frame failed !\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
