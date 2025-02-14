/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate PDMA0 channel 1 get/clear timeout flag with UART1.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_TEST_LENGTH 100
#define PDMA_TIME 0x5555

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (NVT_DCACHE_ON == 1)
    /* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
    uint8_t g_u8Tx_Buffer[DCACHE_ALIGN_LINE_SIZE(PDMA_TEST_LENGTH)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t g_u8Rx_Buffer[DCACHE_ALIGN_LINE_SIZE(PDMA_TEST_LENGTH)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    __attribute__((aligned)) static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
    __attribute__((aligned)) static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];
#endif

static volatile uint32_t u32IsTxTestOver = 0;
static volatile uint32_t u32IsRxTestOver = 0;

NVT_ITCM void PDMA0_IRQHandler(void);
void PDMA_Init(void);

/**
 * @brief       PDMA0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PDMA0 default IRQ, declared in startup_M55M1.c.
 */
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if (status & 0x1)   /* abort */
    {
        printf("target abort interrupt !!\n");

        if (PDMA_GET_ABORT_STS(PDMA0) & 0x1)
        {
            u32IsTxTestOver = 2;
        }

        if (PDMA_GET_ABORT_STS(PDMA0) & 0x2)
        {
            u32IsRxTestOver = 2;
        }

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_GET_ABORT_STS(PDMA0));
    }
    else if (status & 0x2)     /* done */
    {
        if ((PDMA_GET_TD_STS(PDMA0) & (1 << 0)))
        {
            u32IsTxTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA0, (1 << 0));
        }

        if ((PDMA_GET_TD_STS(PDMA0) & (1 << 1)))
        {
            u32IsRxTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA0, (1 << 1));
        }
    }
    else if (status & 0x100)     /* channel 0 timeout */
    {
        u32IsTxTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(PDMA0, 0);
    }
    else if (status & 0x200)     /* channel 1 timeout */
    {
        u32IsRxTestOver = 3;
        PDMA_SetTimeOut(PDMA0, 1, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA0, 1);
        PDMA_SetTimeOut(PDMA0, 1, 0, PDMA_TIME);
    }
    else
    {
        printf("unknown interrupt !!\n");
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    status = PDMA_GET_INT_STATUS(PDMA0);
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable UART1 module clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Select UART1 module clock source as HIRC and UART1 module clock divider as 1 */
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));
    /* Enable PDMA0 clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);
    /* Set multi-function pins for UART1 RXD and TXD */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();
    /* Lock protected registers */
    SYS_LockReg();
}

void PDMA_Init(void)
{
    /* Open PDMA0 Channel */
    PDMA_Open(PDMA0, 1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA0, 1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0, 0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0, 1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, 1, PDMA_WIDTH_8, PDMA_TEST_LENGTH + 1);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0, 0, ((uint32_t)(&g_u8Tx_Buffer[0])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0, 1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t)(&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, 1, PDMA_REQ_SINGLE, 0);
    //Set timeout
    PDMA_SetTimeOut(PDMA0, 1, 1, PDMA_TIME);
    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TIMEOUT);
    NVIC_EnableIRQ(PDMA0_IRQn);
    u32IsRxTestOver = 0;
    u32IsTxTestOver = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, u32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    UART_Open(UART1, 115200);
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------+\n");
    printf("|  PDMA timeout Test                                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo PDMA timeout function.       |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");
    getchar();

    /*
        Using UART1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
        UART1_TX :  Total transfer length =  (PDMA_TEST_LENGTH  ) * 8 bits
        UART1_RX :  Total transfer length =  (PDMA_TEST_LENGTH+1) * 8 bits
    */

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = (uint8_t)i;
        g_u8Rx_Buffer[i] = 0xff;
    }

#if (NVT_DCACHE_ON == 1)
    /*
        Clean the CPU Data cache before starting the DMA transfer.
        This guarantees that the source buffer will be up to date before starting the transfer.
    */
    SCB_CleanDCache_by_Addr(g_u8Tx_Buffer, sizeof(g_u8Tx_Buffer));
    SCB_CleanDCache_by_Addr(g_u8Rx_Buffer, sizeof(g_u8Rx_Buffer));
#endif  // (NVT_DCACHE_ON == 1)

    while (1)
    {
        PDMA_Init();
        UART_PDMA_ENABLE(UART1, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (u32IsTxTestOver == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PDMA Tx Test time-out!\n");
                break;
            }
        }

        if (u32IsTxTestOver == 1)
        {
            printf("UART1 TX transfer done...\n");
        }
        else if (u32IsTxTestOver == 2)
        {
            printf("UART1 TX transfer abort...\n");
        }
        else if (u32IsTxTestOver == 3)
        {
            printf("UART1 TX timeout...\n");
        }

        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (u32IsRxTestOver == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PDMA Rx Test time-out!\n");
                break;
            }
        }

        if (u32IsRxTestOver == 1)
        {
            printf("UART1 RX transfer done...\n");
        }
        else if (u32IsRxTestOver == 2)
        {
            printf("UART1 RX transfer abort...\n");
        }
        else if (u32IsRxTestOver == 3)
        {
            printf("UART1 RX timeout...PDMA timeout test Pass\n");
        }

        UART_PDMA_DISABLE(UART1, UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);
        printf("Please press any key to test again. \n\n");
        getchar();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
