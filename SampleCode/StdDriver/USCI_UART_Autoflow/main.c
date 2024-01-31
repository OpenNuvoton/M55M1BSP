/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data using auto flow control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int32_t g_i32Pointer = 0;
static uint8_t g_u8RecData[RXBUFSIZE] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTest(void);
void USCI_AutoFlow_FunctionTxTest(void);
void USCI_AutoFlow_FunctionRxTest(void);
void SYS_Init(void);
void USCI0_Init(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

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

    /* Enable USCI0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pins for USCI0_DAT0(PA.10), USCI0_DAT1(PA.9), USCI0_CTL0(PC.13) and USCI0_CTL1(PA.8) */
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
    SET_USCI0_CTL1_PA8();
    SET_USCI0_CTL0_PC13();

}


void USCI0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(SYS_USCI0RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Init USCI0 */
    USCI0_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto flow sample function */
    USCI_AutoFlow_FunctionTest();

    printf("\nUSCI UART Sample Program End\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     Pin Configure                                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                              _____  |\n");
    printf("| |      |                                            |     | |\n");
    printf("| |Master|                                            |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1(PA.9)  <==> USCI0_DAT0(PA.10)--|RX   | |\n");
    printf("| |  nCTS|--USCI0_CTL0(PC.13) <==> USCI0_CTL1(PA.8)---|nRTS | |\n");
    printf("| |______|                                            |_____| |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                                |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave. Master will send 1k bytes data       |\n");
    printf("|    to slave. Slave will check if received data is correct   |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    if (u8Item == '0')
        USCI_AutoFlow_FunctionTxTest();
    else
        USCI_AutoFlow_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Master)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTxTest(void)
{
    uint32_t u32Idx;

    /* Enable CTS and RTS autoflow control */
    UUART_EnableFlowCtrl(UUART0);

    /* Send 1k bytes data */
    for (u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        /* Send 1 byte data */
        UUART_WRITE(UUART0, (u32Idx & 0xFF));

        /* Wait if Tx FIFO is full */
        while (UUART_GET_TX_FULL(UUART0));
    }

    printf("\n Transmit Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionRxTest(void)
{
    uint32_t u32Idx, u32Err = 0;

    /* Enable CTS and RTS autoflow control */
    UUART_EnableFlowCtrl(UUART0);

    /* Enable USCI receive end and receive buffer over-run error Interrupt */
    UUART_EnableInt(UUART0, UUART_RXEND_INT_MASK | UUART_BUF_RXOV_INT_MASK);
    NVIC_EnableIRQ(USCI0_IRQn);

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while (g_i32Pointer < RXBUFSIZE);

    /* Compare Data */
    for (u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        if (g_u8RecData[u32Idx] != (u32Idx & 0xFF))
        {
            u32Err = 1;
            break;
        }
    }

    if (u32Err)
        printf("Compare Data Failed\n");
    else
        printf("\n Receive OK & Check OK\n");

    /* Disable USCI interrupt */
    UUART_DisableInt(UUART0, UUART_RXEND_INT_MASK | UUART_BUF_RXOV_INT_MASK);
    NVIC_DisableIRQ(USCI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void USCI0_IRQHandler(void)
{

    volatile uint32_t u32ProtSts = UUART_GET_PROT_STATUS(UUART0);
    volatile uint32_t u32BufSts = UUART_GET_BUF_STATUS(UUART0);
    uint8_t u8InChar = 0xFF;

    if (u32ProtSts & UUART_PROTSTS_RXENDIF_Msk)     /* Receive end interrupt */
    {
        /* Handle received data */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);
        u8InChar = (uint8_t)UUART_READ(UUART0);
        g_u8RecData[g_i32Pointer++] = u8InChar;
    }
    else if (u32BufSts & UUART_BUFSTS_RXOVIF_Msk)     /* Receive buffer over-run error interrupt */
    {
        UUART_CLR_BUF_INT_FLAG(UUART0, UUART_BUFSTS_RXOVIF_Msk);
        printf("\nRx buffer is over-run.");
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
