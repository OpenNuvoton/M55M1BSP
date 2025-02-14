/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to do LPSPI loopback test
 *          in Auto-operation mode when chip enters Power-down mode
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*--------------------------------------------------------------------------*/
/* Define global variables and constants                                    */
/*--------------------------------------------------------------------------*/
#define DATA_COUNT                  32
#define LPTMR0_FREQ                 1000
#define LPSPI_MASTER_TX_DMA_CH      0
#define LPSPI_MASTER_RX_DMA_CH      1
#define LPSPI_OPENED_CH             ((1 << LPSPI_MASTER_TX_DMA_CH) | (1 << LPSPI_MASTER_RX_DMA_CH))

#define TEST_PATTERN                0x55000000
#define LPSPI_CLK_FREQ              2000000

#undef LPPDMA0
#define LPPDMA0                     LPPDMA

//------------------------------------------------------------------------------
#if (NVT_DCACHE_ON == 1)
    // DCache-line aligned buffer for LPSPI Auto Operation mode test
    uint32_t g_au32MasterToSlaveTestPattern[DCACHE_ALIGN_LINE_SIZE(DATA_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE), section(".lpSram")));
    uint32_t g_au32MasterRxBuffer[DCACHE_ALIGN_LINE_SIZE(DATA_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE), section(".lpSram")));
#else
    // Buffer for LPSPI Auto Operation mode test
    uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT] __attribute__((section(".lpSram")));
    uint32_t g_au32MasterRxBuffer[DATA_COUNT] __attribute__((section(".lpSram")));
#endif
volatile uint32_t g_u32WakeupCount = 0;
volatile uint32_t g_u32LPPdmaIntFlag;
volatile uint32_t g_u32Ifr = 0;

//------------------------------------------------------------------------------
NVT_ITCM void LPSPI0_IRQHandler(void)
{
    volatile uint32_t u32Status = 0;
    volatile int32_t i32Timeout = 0xFFFFFF;

    /* for Auto Operation mode test */
    g_u32Ifr = LPSPI0->AUTOSTS;

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    while ((LPSPI0->AUTOSTS != 0) && (--i32Timeout >= 0))
    {
        u32Status = LPSPI0->AUTOSTS;
        LPSPI0->AUTOSTS = u32Status;
    }
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

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

    /* Select HIRC as the clock source of LPSPI0 */
    CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_HIRC, MODULE_NoMsk);

    /* Enable LPSPI module clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Select LPTMR0 module clock source from HIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, MODULE_NoMsk);

    /* Enable LPTMR0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable LPSRAM clock source */
    /* LPPDMA only can access LPSRAM and cannot access normal SRAM. */
    CLK_EnableModuleClock(LPSRAM0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SET_LPSPI0_SS_PA3();
    SET_LPSPI0_CLK_PA2();
    SET_LPSPI0_MISO_PA1();
    SET_LPSPI0_MOSI_PA0();

    /* Clock output HCLK to PD.13 */
    SET_CLKO_PD13();
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_HIRC, 0, CLK_CLKOCTL_DIV1EN_DIV_1);

    /* Disable Auto Operation Clock in Power-down Mode */
    PMC_DISABLE_AOCKPD();
}

void LPTMR0_Init(void)
{
    /* Open LPTMR */
    LPTMR_Open(LPTMR0, LPTMR_ONESHOT_MODE, LPTMR0_FREQ);

    /* Set LPTMR trigger source by time out event */
    LPTMR_SetTriggerSource(LPTMR0, LPTMR_TRGSRC_TIMEOUT_EVENT);

    /* Enable LPTMR clock in power-down mode */
    LPTMR0->CTL |= LPTMR_CTL_PDCLKEN_Msk;

    /* Begin LPTMR trigger function */
    LPTMR0->TRGCTL = LPTMR_TRGCTL_TRGEN_Msk;
}

void LPPDMA_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(SYS_LPPDMA0RST);

    /* Enable PDMA channels */
    LPPDMA_Open(LPPDMA0, LPSPI_OPENED_CH);

    /*=======================================================================
      SPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Incresing
        Destination = SPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, LPPDMA_SAR_INC, (uint32_t)&LPSPI0->TX, LPPDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_LPSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA0, LPSPI_MASTER_TX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    LPPDMA0->LPDSCT[LPSPI_MASTER_TX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = SPI0->RX
        Source Address = Fixed
        Destination = g_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
#if (NVT_DCACHE_ON == 1)
    /* Clean the data cache for the destination buffer of SPI master PDMA RX channel */
    SCB_InvalidateDCache_by_Addr((uint32_t *)&g_au32MasterRxBuffer, sizeof(g_au32MasterRxBuffer));
#endif
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, (uint32_t)&LPSPI0->RX, LPPDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, LPPDMA_DAR_INC);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_LPSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA0, LPSPI_MASTER_RX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    LPPDMA0->LPDSCT[LPSPI_MASTER_RX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Clear transfer done flag */
    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF0_Msk << LPSPI_MASTER_TX_DMA_CH);
    LPPDMA_CLR_TD_FLAG(LPPDMA0, LPPDMA_TDSTS_TDIF0_Msk << LPSPI_MASTER_RX_DMA_CH);
}

void LPSPI_Init(void)
{
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge.
       Set IP clock divider. SPI clock rate = 2 MHz */
    LPSPI_Open(LPSPI0, LPSPI_MASTER, LPSPI_MODE_0, 32, LPSPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    LPSPI_EnableAutoSS(LPSPI0, LPSPI_SS, LPSPI_SS_ACTIVE_LOW);

    /* Select LPSPI Auto Trigger source from LPTMR0 */
    LPSPI_SET_AUTO_TRIG_SOURCE(LPSPI0, LPSPI_AUTOCTL_TRIGSEL_LPTMR0);

    /* Enable Auto Trigger mode */
    LPSPI_ENABLE_AUTO_TRIG(LPSPI0);

    /* Enable Full RX (data recived in TX phase) */
    LPSPI_ENABLE_AUTO_FULLRX(LPSPI0);

    /* Enable TCNT (=0, no RX phase) in RX phase */
    LPSPI_SET_AUTO_RX_TCNT(LPSPI0, DATA_COUNT);

    /* Enable Auto CNT match wake up */
    LPSPI_ENABLE_AUTO_CNT_WAKEUP(LPSPI0);

    /* Clear CNT match wake up flag */
    LPSPI_CLR_AUTO_CNTWK_FLAG(LPSPI0);

    /* Clear CNT match interrupt flag */
    LPSPI_CLR_AUTO_CNTWK_FLAG(LPSPI0);

    /* Enable Auto CNT match interrupt */
    LPSPI_ENABLE_AUTO_CNT_INT(LPSPI0);

    /* Enable Auto mode */
    LPSPI_ENABLE_AUTO(LPSPI0);

    /* Enable LPSPI0 interrupt */
    NVIC_EnableIRQ(LPSPI0_IRQn);
}

void AutoOperation_FunctionTest()
{
    uint32_t u32i, u32DataCount, u32TotalRxCount;
    uint32_t u32PdmaDoneFlag, u32Status;

    g_u32WakeupCount = 0;

    /* Init LPSPI runs in Auto Operation Mode */
    LPSPI_Init();

    /* Init LPTMR */
    LPTMR0_Init();

    while (1)
    {
#if (NVT_DCACHE_ON == 1)
        /* If DCACHE is enabled, clean the data cache for the two buffers before writing to them and enabling DMA */
        /* This is to ensure that the data written to the cache is actually written to the memory */
        SCB_CleanDCache_by_Addr((uint32_t *)&g_au32MasterToSlaveTestPattern, sizeof(g_au32MasterToSlaveTestPattern));
        SCB_CleanDCache_by_Addr((uint32_t *)&g_au32MasterRxBuffer, sizeof(g_au32MasterRxBuffer));
#endif

        /* Source data initiation */
        for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
        {
            g_au32MasterToSlaveTestPattern[u32DataCount] = ((TEST_PATTERN | (u32DataCount + 1)) + g_u32WakeupCount);
            g_au32MasterRxBuffer[u32DataCount] = 0x00;
        }

        LPPDMA_Init();

        printf("\nPower down and wait LPPDMA to wake up CPU ...\n\n");
        UART_WAIT_TX_EMPTY(DEBUG_PORT);

        /* Clear all wake-up status flags */
        //CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        g_u32LPPdmaIntFlag = 0;

        LPTMR_Start(LPTMR0);

        SYS_UnlockReg();

        /* Switch SCLK clock source to HIRC and divide 1 */
        CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

        PMC_SetPowerDownMode(PMC_NPD1, PMC_PLCTL_PLSEL_PL1);
        PMC_PowerDown();

        /* Switch SCLK clock source to PLL0 and divide 1 */
        CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

        printf("Woken up %d times !!\n", ++g_u32WakeupCount);

        SYS_LockReg();

        /* g_u32Ifr is set in LPSPI0 interrupt service routine */
        while (g_u32Ifr == 0);

        u32Status = LPSPI0->AUTOSTS;
        LPSPI0->AUTOSTS = u32Status;

        if ((g_u32Ifr & (LPSPI_AUTOSTS_CNTIF_Msk | LPSPI_AUTOSTS_CNTWKF_Msk))
                != (LPSPI_AUTOSTS_CNTIF_Msk | LPSPI_AUTOSTS_CNTWKF_Msk))
        {
            printf("Some errors happened, LPSPI->AUTOSTS=0x%x \n", g_u32Ifr);

            while (1);
        }

        /* Check transfer result */
        u32PdmaDoneFlag = LPPDMA_GET_TD_STS(LPPDMA0);

        /* check the master's TX DMA interrupt flag */
        if ((u32PdmaDoneFlag & (1 << LPSPI_MASTER_TX_DMA_CH)) == 0) while (1);

        LPPDMA_CLR_TD_FLAG(LPPDMA0, (1 << LPSPI_MASTER_TX_DMA_CH));

        /* check the master's RX DMA interrupt flag */
        if ((u32PdmaDoneFlag & (1 << LPSPI_MASTER_RX_DMA_CH)) == 0) while (1);

        LPPDMA_CLR_TD_FLAG(LPPDMA0, (1 << LPSPI_MASTER_RX_DMA_CH));

        printf("*** Since LPSPI TX doesn't send data in RX phase, RX pin cannot get last one sample in this example code. ***\n\n");

        /* u32TotalRxCount is the total received data number in both TX phase (by FULLRX enabled) and RX phase */
        u32TotalRxCount = DATA_COUNT;

        for (u32i = 0; u32i < u32TotalRxCount; u32i++)
        {
            printf("    [%2d]: %08X\n", u32i, g_au32MasterRxBuffer[u32i]);
        }
    }   /* end of while(1) */
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\n");
    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------------------+\n");
    printf("|         M55M1 LPSPI Auto Operation Mode Sample Code                |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates LPSPI0 self loop back data transfer in Auto Operation Mode,\n");
    printf("  and its procedure is listed below. \n");
    printf(" 1. Initialize LPTMR0, LPPDMA and LPSPI0. \n");
    printf("    LPSPI0 is configured to Master mode with data width is 32 bits.\n");
    printf("    For loop back test, its I/O connection: LPSPI0_MOSI(PA.0) <--> LPSPI0_MISO(PA.1) \n");
    printf(" 2. Let system enter Powerr Mode. \n");
    printf(" 3. LPTMR0 will trigger LPSPI0 to do 32-sample data transfer per one second. After finishing data transfer, \n");
    printf("    system will be woken up and the received data can be checked. \n");
    printf(" 4. The above step-2 and step-3 will be executed repeatedly. \n\n");
    printf("Please hit any key to start test.\n\n");
    getchar();

    AutoOperation_FunctionTest();

    /* Lock protected registers */
    SYS_LockReg();

    printf("Exit LPSPI Auto-operation sample code\n");

    while (1);
}
