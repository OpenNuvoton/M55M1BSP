/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo how to setup CCAP motion detection function to wakeup system
 *           when motion is detected in specified regions under power down mode
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sensor.h"

#define CCAP_OUTPUT_WIDTH       320
#define CCAP_OUTPUT_HEIGHT      240
#define WK_REINIT_SENSOR        0

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
    uint32_t u32IntStatus;

    u32IntStatus = CCAP_GET_INT_STS();

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

    if (CCAP_IsIntEnabled(CCAP_INT_MDIEN_MODE1_ENABLE | CCAP_INT_MDIEN_MODE2_ENABLE) && (u32IntStatus & CCAP_INTSTS_MDINTF_Msk))
    {
        CCAP_CLR_INT_FLAG(CCAP_INTSTS_MDINTF_Msk);

        if (CCAP_GET_WAKEUP_FUNC() && CCAP_GET_WAKEUP_FLAG())
        {
            LPTMR_Stop(LPTMR0);
            CCAP_CLR_WAKEUP_FLAG();
            printf("Wakeup by CCAP motion detection (u32IntStatus: 0x%X).\n", u32IntStatus);
        }
    }

    CCAP_EnableUpdate();
    // Loop until either the flag is cleared or timeout occurs
    i32TimeoutCnt = 1000 * CyclesPerUs;

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

NVT_ITCM void LPTMR0_IRQHandler(void)
{
    int32_t  i32TimeoutCnt;

    if (LPTMR_GetWakeupFlag(LPTMR0))
    {
        printf("LPTMR TWKF ");
    }

    /* Clear wake up flag */
    LPTMR_ClearWakeupFlag(LPTMR0);

    if (LPTMR_GetIntFlag(LPTMR0))
    {
        printf("LPTMR TIF\n");
    }

    /* Clear interrupt flag */
    LPTMR_ClearIntFlag(LPTMR0);

    // Loop until either the flag is cleared or timeout occurs
    i32TimeoutCnt = 1000 * CyclesPerUs;

    while (1)
    {
        // Check flag is cleared
        if ((LPTMR_GetWakeupFlag(LPTMR0) == 0) && (LPTMR_GetIntFlag(LPTMR0) == 0))
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

    printf("CCAP   engine clock: %d Hz (Div: %d)\n", u32CCAP_Clk, i32Div);
    printf("Target sensor clock: %d Hz\n", u32SensorFreq);
    printf("Actual sensor clock: %d Hz\n", u32CCAP_Clk / (i32Div + 1));
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable internal medium speed RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_MIRCEN_Msk);
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_MIRCSTB_Msk);

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

    /* Enable CCAP Clock */
    CLK_EnableModuleClock(CCAP0_MODULE);
    /* Reset IP */
    SYS_ResetModule(SYS_CCAP0RST);

    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);

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

int32_t ConfigLPTMR(LPTMR_T *psLPTMR, uint32_t u32PreiodInMS)
{
    printf("LPTMR trigger period: %d ms\n", u32PreiodInMS);
    SYS_UnlockReg();

    LPTMR_Stop(psLPTMR);
    LPTMR_Open(psLPTMR, LPTMR_PERIODIC_MODE, (1000 / u32PreiodInMS));
    LPTMR_EnablePDCLK(psLPTMR);
    LPTMR_SetTriggerTarget(psLPTMR, TTMR_TRGEN);
    LPTMR_Start(psLPTMR);
    return 0;
}

int32_t PacketMotionDetection(S_SENSOR_INFO *psSensorInfo)
{
    int32_t  i;

    /* Initialize sensor and set sensor output YUV422 format */
    if (psSensorInfo->pfnInitSensor((uint32_t)psSensorInfo) == FALSE)
    {
        printf("Init sensor failed !\n");
        return -1;
    }

    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CCAP_SetCroppingWindow(0, 0, psSensorInfo->m_u16Height, psSensorInfo->m_u16Width);

    /* Set System Memory Packet Base Address Register */
    CCAP_SetPacketBuf((uint32_t)g_au8FrameBuffer);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor Format and Order */
    CCAP_OpenPipes(psSensorInfo->m_u32Polarity, psSensorInfo->m_u32InputFormat, CCAP_PKT_OUTFMT_YUV422, CCAP_PLN_DISABLED);

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CCAP_SetPacketScaling(CCAP_OUTPUT_HEIGHT, psSensorInfo->m_u16Height, CCAP_OUTPUT_WIDTH, psSensorInfo->m_u16Width);

    /* Set Packet Frame Output Pixel Stride Width */
    CCAP_SetPacketStride(CCAP_OUTPUT_WIDTH);

    /* Configure the control register about Motion detection feature */
    CCAP_MD_ENABLE_WINDOW(0xFFFF);
    /* Set global motion detection threshold, maximum value is 0x12AD40
     *   If (sum of all windows diff of two frame) > (global motion detection threshold),
         CCAP will trigger MD1 interrupt (CCAP_INTSTS_MDINTF_MODE1_Msk).
     */
    CCAP_MD_SET_TOTAL_THRESHOLD(0x2000 * CCAP_MD_WINDOW_CNT);

    /* Set window motion detection threshold, maximum value is 0x12AD4
     *   If (window diff of two frame) > (window motion detection threshold), overflow window count is increment by 1.
     */
    for (i = 0; i < CCAP_MD_WINDOW_CNT; i++)
        CCAP_MD_SET_WIN_THRESHOLD(i, 0x2000);

    /* Set overflow window count threshold, maximum value is 15.
     *   If overflow window count > overflow window count threshold,
     *   CCAP will trigger MD2 interrupt (CCAP_INTSTS_MDINTF_MODE2_Msk).
     */
    CCAP_MD_SET_OVERFLOW_WIN_THRESHOLD(4);
    /* Configure CCAPEN(CCAP_CTL[0]) to disable the capture interface. */
    CCAP_Close();

    SYS_UnlockReg();
    PMC_SetCCAP_SRAMPowerMode(PMC_SRAM_NORMAL);
    /* Enable CCAP Motion Detect Interrupt */
    CCAP_EnableInt(CCAP_INTEN_MDIEN_Msk);
    /* Enable External CAP Interrupt */
    NVIC_EnableIRQ(CCAP_IRQn);

    return 0;
}

int32_t PacketFormatDownScale(S_SENSOR_INFO *psSensorInfo)
{
    uint32_t u32Frame;

#if (WK_REINIT_SENSOR == 1)

    /* Initialize sensor and set output format as YUV422 */
    if (psSensorInfo->pfnInitSensor((uint32_t)psSensorInfo) == FALSE)
    {
        printf("Init sensor failed !\n");
        return -1;
    }

#endif

    /* Enable External CCAP Interrupt */
    NVIC_EnableIRQ(CCAP_IRQn);

    /* Enable CCAP Frame End Interrupt */
    CCAP_EnableInt(CCAP_INT_VIEN_ENABLE);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor Format and Order */
    CCAP_OpenPipes(psSensorInfo->m_u32Polarity, psSensorInfo->m_u32InputFormat, CCAP_PKT_OUTFMT_YUV422, CCAP_PLN_DISABLED);

    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CCAP_SetCroppingWindow(0, 0, psSensorInfo->m_u16Height, psSensorInfo->m_u16Width);

    /* Set System Memory Packet Base Address Register */
    CCAP_SetPacketBuf((uint32_t)g_au8FrameBuffer);

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CCAP_SetPacketScaling(CCAP_OUTPUT_HEIGHT, psSensorInfo->m_u16Height, CCAP_OUTPUT_WIDTH, psSensorInfo->m_u16Width);

    /* Set Packet Frame Output Pixel Stride Width */
    CCAP_SetPacketStride(CCAP_OUTPUT_WIDTH);

    u32Frame = g_u32FramePass;
    printf("u32Frame: %d, g_u32FramePass: %d\n", u32Frame, g_u32FramePass);
    /* Stop Capture Interface after one frame captured */
    CCAP_Stop(TRUE);

    while (1)
    {
        if (u32Frame != g_u32FramePass)
        {
            u32Frame = g_u32FramePass;
            printf("Captured one frame.\n");
            break;
        }
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t  i;
    uint32_t u32Threshold;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+-------------------------------------------+\n");
    printf("|  M55M1 CCAP Motion Detection Sample Code  |\n");
    printf("+-------------------------------------------+\n");

    while (1)
    {
        SYS_UnlockReg();

        PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL0);
        /* Init CCAP clock and Sensor clock */
        CCAP_SetFreq(CLK_CCAPSEL_CCAP0SEL_HIRC, 12000000);

        /* Config CCAP motion detection */
        if (PacketMotionDetection(&g_sSensorHM1055_VGA_YUV422) != 0)
        {
            printf("Test motion detection failed !\n");
            break;
        }

        /* Config LPTMR0 to trigger CCAP motion detection per 100 ms */
        ConfigLPTMR(LPTMR0, 100);
        CCAP_MD_SET_TRIG_SRC(CCAP_MD_TRIG_LPTMR0, TRUE);
        printf("Press any key to enter power down\n");
        getchar();
        /* Keep MIRC and HIRC enabled in power down mode for TTMR/LPTMR auto operation. */
        PMC_DISABLE_AOCKPD();
        printf("Power down ...\n");
        /* Check if all the debug messages are finished */
        UART_WAIT_TX_EMPTY(DEBUG_PORT);
        /* Switch SCLK to HIRC when power down */
        CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_HIRC, CLK_APLLCTL_APLLSRC_HIRC, 0);

        /* Enter to Power-down mode */
        PMC_PowerDown();

        /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
        CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
        /* Init CCAP clock and Sensor clock */
        CCAP_SetFreq(CLK_CCAPSEL_CCAP0SEL_HCLK2, 24000000);

        // Capture 1 frame after wakeup
        if (PacketFormatDownScale(&g_sSensorHM1055_VGA_YUV422) != 0)
            printf("Capture frame failed !\n");

        printf("\nMotion Detection Status:\n");
        u32Threshold = CCAP_MD_GET_TOTAL_THRESHOLD();
        printf("  Total sum of diff: 0x%08X (Threshold: 0x%08X)\n", CCAP_MD_GET_TOTAL_SAD(), u32Threshold);
        u32Threshold = CCAP_MD_GET_OVERFLOW_WIN_THRESHOLD();
        printf("  Diff window count: %d (Threshold: %d)\n", CCAP_MD_GET_OVERFLOW_WIN_CNT(), u32Threshold);

        for (i = 0; i < CCAP_MD_WINDOW_CNT; i++)
        {
            uint32_t u32WinSAD;

            if (i % 4 == 0)
            {
                printf("|--------------|--------------|--------------|--------------|\n");
                printf("|");
            }

            u32WinSAD = CCAP_MD_GET_WIN_SAD(i);

            if (u32WinSAD > CCAP_MD_GET_WIN_THRESHOLD(i))
                printf(" %s[*] 0x%05X |", (CCAP_MD_IS_WINDOW_ENABLED(1 << i)) ? "V" : "X", u32WinSAD);
            else
                printf(" %s[ ] 0x%05X |", (CCAP_MD_IS_WINDOW_ENABLED(1 << i)) ? "V" : "X", u32WinSAD);

            if ((i + 1) % 4 == 0)
                printf("\n");
        }

        printf("|--------------|--------------|--------------|--------------|\n");
        printf("  V: Window Enabled, X: Window Disabled\n  *: Window Diff > Window Threshold\n\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
