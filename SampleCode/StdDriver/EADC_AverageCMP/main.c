/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to compare average conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// DAT gets unexpected data during conversion.
// If you want to avoid this issue, please use PDMA.
#define _USE_PDMA 1

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32IsTestOver = 0;

#if (NVT_DCACHE_ON == 1 && _USE_PDMA == 1)
/* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
int32_t g_i32ConversionData[DCACHE_ALIGN_LINE_SIZE(1)] __attribute__((aligned(DCACHE_LINE_SIZE))) = {0};
#else
int32_t g_i32ConversionData[1] __attribute__((aligned)) = {0};
#endif
uint32_t g_u32SampleModuleNum = 1; /* Use Sample Module 1 */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

#if(_USE_PDMA==1)
/*---------------------------------------------------------------------------------------------------------*/
/* PDMA interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");

    __NOP();
}

#else
/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32IsTestOver = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Enable APLL1 180MHz clock for maximum EADC clock frequency */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ, CLK_APLL1_SELECT);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select EADC peripheral clock source(maximum clock frequency = divider 1) */
    CLK_SetModuleClock(EADC0_MODULE, CLK_EADCSEL_EADC0SEL_APLL1_DIV2, CLK_EADCDIV_EADC0DIV(15));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);
#if(_USE_PDMA==1)

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);
#endif
    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting */
    SetDebugUartCLK();

    /* To run the CPU at 220 MHz, the power level must be set to PL0. */
    PMC_SetPowerLevel(PMC_PLCTL_PLSEL_PL0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB.0 - PB.1 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.1 ADC analog input pins. */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();
    /* Disable the PB.0 - PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);
}

#if(_USE_PDMA==1)

void PDMA_Init()
{
    /* Configure PDMA peripheral mode form EADC to memory */
    /* Open Channel 2 */
    PDMA_Open(PDMA0, BIT2);

    /* transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_16, 1);

    /* Set source address as EADC data register(no increment) and destination address as g_i32ConversionData array(increment) */
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)&EADC0->DAT[g_u32SampleModuleNum], PDMA_SAR_FIX, (uint32_t)g_i32ConversionData, PDMA_DAR_INC);

    /* Set PDMA as single request type for EADC */
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_SINGLE, PDMA_BURST_1);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_EADC0_RX, FALSE, 0);

    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void ReloadPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_16, 1);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_EADC0_RX, FALSE, 0);
}

#endif
void EADC_FunctionTest()
{
    uint8_t u8Option;
    int32_t i32ConversionData, i32Target;

    uint32_t u32ChannelNum;
    uint32_t u32ModuleMask;

    u32ModuleMask = (BIT0 << g_u32SampleModuleNum);

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|             EADC Average with sofeware compare sample code           |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Basic EADC conversion (channel 0 only)\n");
        printf("  [2] Basic EADC conversion (channel 1 only)\n");
        printf("  Other keys: exit EADC test\n");
        u8Option = getchar();

        if (u8Option == '1')
            u32ChannelNum = 0;
        else if (u8Option == '2')
            u32ChannelNum = 1;
        else
            break;  /* exit while loop */

        /* Configure the sample module(g_u32SampleModuleNum) for analog input channel(u32ChannelNum) and software trigger source */
        EADC_ConfigSampleModule(EADC0, g_u32SampleModuleNum, EADC_SOFTWARE_TRIGGER, u32ChannelNum);

        /* Set sample module external sampling time to 10 */
        EADC_SetExtendSampleTime(EADC0, g_u32SampleModuleNum, 10);

        /* Enable Accumulate feature */
        EADC_ENABLE_ACU(EADC0, g_u32SampleModuleNum, EADC_MCTL1_ACU_32);

        /* Enable Average feature */
        EADC_ENABLE_AVG(EADC0, g_u32SampleModuleNum);
#if(_USE_PDMA==1)
        ReloadPDMA();
        EADC_ENABLE_SAMPLE_MODULE_PDMA(EADC0, u32ModuleMask);
#else
        uint32_t u32IntNum = 0;
        uint32_t u32IntMask = (BIT0 << u32IntNum);
        /* Clear the A/D ADINT0 interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, u32IntMask);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_EnableIRQ(EADC00_IRQn);
#endif
        /* Reset the interrupt indicator and trigger sample module to start A/D conversion */
        g_u32IsTestOver = 0;
        EADC_START_CONV(EADC0, u32ModuleMask);

        /* Wait interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
        while (g_u32IsTestOver == 0);

        /* Disable Average feature */
        EADC_DISABLE_ACU(EADC0, g_u32SampleModuleNum);

        /* Disable Accumulate feature */
        EADC_DISABLE_AVG(EADC0, g_u32SampleModuleNum);
#if(_USE_PDMA==1)
        EADC_DISABLE_SAMPLE_MODULE_PDMA(EADC0, u32ModuleMask);
#if (NVT_DCACHE_ON == 1)
        /* Clean the data cache for the destination buffer of EADC PDMA RX channel */
        SCB_InvalidateDCache_by_Addr(g_i32ConversionData, sizeof(g_i32ConversionData));
#endif
        i32ConversionData = g_i32ConversionData[0];
#else
        /* Disable the sample module interrupt. */
        EADC_DISABLE_INT(EADC0, u32IntMask);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, u32IntNum, u32ModuleMask);
        NVIC_DisableIRQ(EADC00_IRQn);
        /* Get the conversion result of the sample module */
        i32ConversionData = EADC_GET_CONV_DATA(EADC0, g_u32SampleModuleNum);
#endif
        /* EADC oversampling mode or averaging mode can not be monitored by compare and window compare functions */
        /* Implement software compare to replace hardware compare */
        printf("Conversion result of channel %u: 0x%X (%d)\n", u32ChannelNum, i32ConversionData, i32ConversionData);
        i32Target = 0x600;

        if (i32ConversionData >= i32Target)
        {
            printf("The conversion result of channel %u is >= 0x%03X\n", u32ChannelNum, i32Target);
        }
        else
        {
            printf("The conversion result of channel %u is < 0x%03X\n", u32ChannelNum, i32Target);
        }
    }   /* end of while(1) */
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

#if !(defined(DEBUG_ENABLE_SEMIHOST))
    /* Init Debug UART for printf */
    InitDebugUart();
#endif

#if(_USE_PDMA==1)
    /* Init PDMA for EADC */
    PDMA_Init();
#endif
    printf("\nSystem clock rate: %d Hz", SystemCoreClock);
    /* EADC function test */
    EADC_FunctionTest();
#if(_USE_PDMA==1)
    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA0_MODULE);
#endif
    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
