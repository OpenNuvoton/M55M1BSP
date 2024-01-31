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

#define CCAP_OUTPUT_WIDTH       160
#define CCAP_OUTPUT_HEIGHT      120
uint8_t g_au8FrameBuffer[DCACHE_ALIGN_LINE_SIZE(CCAP_OUTPUT_WIDTH * CCAP_OUTPUT_HEIGHT * 2)] __ALIGNED(DCACHE_LINE_SIZE);

/*------------------------------------------------------------------------------------------*/
/* To run CCAPInterruptHandler, when CCAP frame end interrupt                               */
/*------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32FramePass = 0;

/*------------------------------------------------------------------------------------------*/
/*  CCAP_IRQHandler                                                                         */
/*------------------------------------------------------------------------------------------*/
NVT_ITCM void CCAP_IRQHandler(void)
{
    uint32_t u32IntStatus = CCAP_GET_INT_STS();

    if (CCAP_IsIntEnabled(CCAP_INT_VIEN_ENABLE) && (u32IntStatus & CCAP_INTSTS_VINTF_Msk) == CCAP_INTSTS_VINTF_Msk)
    {
#ifdef NVT_DCACHE_ON
        /* Invalidate data cache of received frame buffer.  */
        SCB_InvalidateDCache_by_Addr(g_au8FrameBuffer, sizeof(g_au8FrameBuffer));
#endif
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
    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32IntStatus = CCAP_GET_INT_STS();
}

void CCAP_SetFreq(uint32_t u32ModFreqKHz, uint32_t u32SensorFreq)
{
    int32_t i32Div;

    NVT_UNUSED(i32Div);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable CCAP Clock */
    CLK_EnableModuleClock(CCAP0_MODULE);

    /* Reset IP */
    SYS_ResetModule(SYS_CCAP0RST);

    /* Specified sensor clock */
    CLK_SetModuleClock(CCAP0_MODULE, CLK_CCAPSEL_CCAP0SEL_HIRC, 0);
    /* Not support in TESTCHIP_ONLY
        i32Div= (CLK_GetHCLK2Freq() / u32SensorFreq) - 1;
        if(i32Div < 0)
            i32Div = 0;
        CLK->VSENSEDIV = (CLK->VSENSEDIV & ~CLK_VSENSEDIV_VSENSEDIV_Msk) | (i32Div << CLK_VSENSEDIV_VSENSEDIV_Pos);
     * Not support in TESTCHIP_ONLY */

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t PacketFormatDownScale(S_SENSOR_INFO *psSensorInfo)
{
    uint32_t u32Frame;

    /* Initialize sensor and set output format as YUV422 */
    if (psSensorInfo->pfnInitSensor(0) == FALSE)
    {
        printf("Init sensor failed !\n");
        return -1;
    }

    /* Enable External CCAP Interrupt */
    NVIC_EnableIRQ(CCAP_IRQn);

    /* Enable CCAP Frame End Interrupt */
    CCAP_EnableInt(CCAP_INT_VIEN_ENABLE);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor Format and Order */
    CCAP_Open(psSensorInfo->m_u32Polarity, psSensorInfo->m_u32InputFormat, CCAP_PAR_OUTFMT_YUV422);

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
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);

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
    SET_CCAP_HSYNC_PG13();

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
    CCAP_SetFreq(12000000, 12000000);

    /* Using Packet format to Image down scale */
    if (PacketFormatDownScale(&g_sSensorHM1055) != 0)
        printf("Capture frame failed !\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
