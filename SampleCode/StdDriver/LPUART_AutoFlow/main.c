/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data using auto flow control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define RXBUFSIZE 256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32pointer = 0;
uint8_t g_u8RecData[RXBUFSIZE] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void);
void AutoFlow_FunctionTxTest(void);
void AutoFlow_FunctionRxTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /* Select LPUART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, CLK_LPUARTSEL_LPUART0SEL_HIRC, CLK_LPUARTDIV_LPUART0DIV(1));

    /* Enable LPUART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PA multi-function pins for LPUART0 TXD and RXD and CTS and RTS */
    SET_LPUART0_RXD_PA0();
    SET_LPUART0_TXD_PA1();
    SET_LPUART0_nRTS_PB10();
    SET_LPUART0_nCTS_PB11();


}


/*---------------------------------------------------------------------------------------------------------*/
/* Init LPUART0                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_Init(void)
{
    /* Reset Low Power UART0 */
    SYS_ResetModule(SYS_LPUART0RST);

    /* Configure Low Power UART0 and set Low Power UART0 Baudrate */
    LPUART_Open(LPUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Init LPUART0 */
    LPUART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------+\n");
    printf("|  Auto-Flow function test |\n");
    printf("+--------------------------+\n");

    /* LPUART auto flow sample function */
    AutoFlow_FunctionTest();

    printf("\nLPUART Sample Program End\n");

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  ______                                                _____  |\n");
    printf("| |      |                                              |     | |\n");
    printf("| |Master|--LPUART0_TXD(PA.1)  <==> LPUART0_RXD(PA.0) --|Slave| |\n");
    printf("| |      |--LPUART0_nCTS(PB.11)<==> LPUART0_nRTS(PB.10)-|     | |\n");
    printf("| |______|                                              |_____| |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n");

    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                                  |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  Description :                                                |\n");
    printf("|    The sample code needs two boards. One is Master and        |\n");
    printf("|    the other is slave. Master will send 1k bytes data         |\n");
    printf("|    to slave. Slave will check if received data is correct     |\n");
    printf("|  Please select Master or Slave test                           |\n");
    printf("|  [0] Master    [1] Slave                                      |\n");
    printf("+---------------------------------------------------------------+\n");
    u8Item = getchar();

    if (u8Item == '0')
        AutoFlow_FunctionTxTest();
    else
        AutoFlow_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Master)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTxTest(void)
{
    uint32_t u32i;

    /* Enable RTS and CTS autoflow control */
    LPUART_EnableFlowCtrl(LPUART0);

    /* Send 1k bytes data */
    for (u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        /* Send 1 byte data */
        LPUART_WRITE(LPUART0, u32i & 0xFF);

        /* Wait if Tx FIFO is full */
        while (LPUART_IS_TX_FULL(LPUART0));
    }

    printf("\n Transmit Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionRxTest(void)
{
    uint32_t u32i;

    /* Enable RTS and CTS autoflow control */
    LPUART_EnableFlowCtrl(LPUART0);

    /* Set RTS Trigger Level as 8 bytes */
    LPUART0->FIFO = (LPUART0->FIFO & (~LPUART_FIFO_RTSTRGLV_Msk)) | LPUART_FIFO_RTSTRGLV_8BYTES;

    /* Set RX Trigger Level as 8 bytes */
    LPUART0->FIFO = (LPUART0->FIFO & (~LPUART_FIFO_RFITL_Msk)) | LPUART_FIFO_RFITL_8BYTES;

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    LPUART_SetTimeoutCnt(LPUART0, 0x3E);

    /* Enable RDA and RTO Interrupt */
    NVIC_EnableIRQ(LPUART0_IRQn);
    LPUART_EnableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_RXTOIEN_Msk));

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while (g_i32pointer < RXBUFSIZE);

    /* Compare Data */
    for (u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        if (g_u8RecData[u32i] != (u32i & 0xFF))
        {
            printf("Compare Data Failed\n");

            while (1);
        }
    }

    printf("\n Receive OK & Check OK\n");

    /* Disable RDA and RTO Interrupt */
    LPUART_DisableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_RXTOIEN_Msk));

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART0 interrupt event                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPUART0_IRQHandler(void)
{
    uint8_t u8InChar = 0xFF;

    /* Rx Ready or Time-out INT */
    if (LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_RDAINT_Msk | LPUART_INTSTS_RXTOINT_Msk))
    {
        /* Read data until RX FIFO is empty */
        while (LPUART_GET_RX_EMPTY(LPUART0) == 0)
        {
            u8InChar = LPUART_READ(LPUART0);
            g_u8RecData[g_i32pointer++] = u8InChar;
        }
    }

    if (LPUART0->FIFOSTS & (LPUART_FIFOSTS_BIF_Msk | LPUART_FIFOSTS_FEF_Msk | LPUART_FIFOSTS_PEF_Msk | LPUART_FIFOSTS_RXOVIF_Msk))
    {
        LPUART_ClearIntFlag(LPUART0, (LPUART_INTSTS_RLSINT_Msk | LPUART_INTSTS_BUFERRINT_Msk));
    }
}



/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
