/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Configure QSPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip QSPI Slave device with FIFO mode. This sample
 *           code needs to work with QSPI_PDMA_Slave sample code.
 *
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define QSPI_MASTER_TX_DMA_CH   0
#define TEST_COUNT              64

//------------------------------------------------------------------------------
/* Global variable declaration */
/* Buffer for SPI0 data transfer with PDMA */
#if (NVT_DCACHE_ON == 1)
/* DCache-line aligned buffer for SPI0 data transfer with PDMA */
uint32_t g_au32MasterToSlaveTestPattern[DCACHE_ALIGN_LINE_SIZE(TEST_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE))) = {0};
#else
/* Buffer for SPI0 data transfer with PDMA when DCache is disabled */
uint32_t g_au32MasterToSlaveTestPattern[TEST_COUNT];
#endif

//------------------------------------------------------------------------------
/* Function prototype declaration */
void SYS_Init(void);
void QSPI_Init(void);
void QSPI_Master_Send(void);

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

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_QSPISEL_QSPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable QSPI0 peripheral clock */
    CLK_EnableModuleClock(QSPI0_MODULE);
    /* Enable PDMA0 peripheral clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup QSPI0 multi-function pins */
    SET_QSPI0_SS_PA3();
    SET_QSPI0_CLK_PA2();
    SET_QSPI0_MOSI0_PA0();
    SET_QSPI0_MISO0_PA1();
    SET_QSPI0_MOSI1_PA4();
    SET_QSPI0_MISO1_PA5();

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk);

    /* Enable QSPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0x3F, GPIO_SLEWCTL_FAST0);
}

void QSPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure QSPI0 */
    /* Enable QSPI0 Quad output mode */
    QSPI_ENABLE_QUAD_OUTPUT_MODE(QSPI0);
    /* Configure QSPI0 as a master, SPI clock rate 1MHz,
       clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    QSPI_Open(QSPI0, SPI_MASTER, SPI_MODE_0, 32, 1000000);
    /* Enable the automatic hardware slave selection function. Select the QSPI0_SS pin and configure as low-active. */
    QSPI_EnableAutoSS(QSPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
}

void QSPI_Master_Send(void)
{
    uint32_t u32DataCount;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;

    printf("\nQSPI0 Master Quad mode with PDMA ");

    /* Source data initiation */
    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = (0x55000000 | (u32DataCount + 1));
    }

    /* Enable PDMA channel */
    PDMA_Open(PDMA0, (1 << QSPI_MASTER_TX_DMA_CH));

    /*=======================================================================
      SPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Increasing
        Destination = QSPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
#if (NVT_DCACHE_ON == 1)
    /* If DCACHE is enabled, clean the data cache for the master to slave test pattern before using it */
    /* This is to ensure that the data written to the cache is actually written to the memory */
    SCB_CleanDCache_by_Addr((uint32_t *)&g_au32MasterToSlaveTestPattern, sizeof(g_au32MasterToSlaveTestPattern));
#endif
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, QSPI_MASTER_TX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, QSPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, QSPI_MASTER_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, QSPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[QSPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable SPI master DMA function */
    QSPI_TRIGGER_TX_PDMA(QSPI0);

    i32Err = 0;

    while (1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA0);

        /* Check the PDMA transfer done interrupt flag */
        if (u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done flags */
            if ((PDMA_GET_TD_STS(PDMA0) & (1 << QSPI_MASTER_TX_DMA_CH)) == (1 << QSPI_MASTER_TX_DMA_CH))
            {
                /* Clear the PDMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA0, (1 << QSPI_MASTER_TX_DMA_CH));

                /* Disable QSPI master's PDMA transfer function */
                QSPI_DISABLE_TX_PDMA(QSPI0);

                break;
            }
        }

        /* Check the DMA transfer abort interrupt flag */
        if (u32RegValue & PDMA_INTSTS_ABTIF_Msk)
        {
            /* Get the target abort flag */
            u32Abort = PDMA_GET_ABORT_STS(PDMA0);
            /* Clear the target abort flag */
            PDMA_CLR_ABORT_FLAG(PDMA0, u32Abort);
            i32Err = 1;
            break;
        }

        /* Check the DMA time-out interrupt flag */
        if (u32RegValue & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))
        {
            /* Clear the time-out flag */
            PDMA0->INTSTS = u32RegValue & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk);
            i32Err = 1;
            break;
        }
    }

    /* Disable all PDMA channels */
    PDMA_Close(PDMA0);

    if (i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init SPI */
    QSPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  QSPI master mode + PDMA Sample Code         |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure QSPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("QSPI controller will enable QUAD mode and transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("The I/O connection for these two QSPI0:\n");
    printf("    QSPI0_SS  (PA3)  <--> QSPI0_SS  (PA3)\n    QSPI0_CLK(PA2)  <--> QSPI0_CLK(PA2)\n");
    printf("    QSPI0_MISO0(PA1) <--> QSPI0_MISO0(PA1)\n    QSPI0_MOSI0(PA0) <--> QSPI0_MOSI0(PA0)\n\n");
    printf("    QSPI0_MISO1(PA5) <--> QSPI0_MISO1(PA5)\n    QSPI0_MOSI1(PA4) <--> QSPI0_MOSI1(PA4)\n\n");
    printf("Press any key to start transmission after QSPI_PDMA_Slave is running...");
    getchar();
    printf("\n");

    QSPI_Master_Send();

    printf("\n\nExit QSPI driver sample code.\n");

    /* Close QSPI0 */
    QSPI_Close(QSPI0);

    while (1);
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
