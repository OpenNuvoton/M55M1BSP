/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive UART data in UART IrDA mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"




/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/*                                         Define functions prototype                                      */
/*---------------------------------------------------------------------------------------------------------*/

void IrDA_FunctionTxTest(void);
void IrDA_FunctionRxTest(void);
void IrDA_FunctionTest(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif
/*---------------------------------------------------------------------------------------------------------*/
/*                                    Init System Clock                                                    */
/*---------------------------------------------------------------------------------------------------------*/
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

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART1 clock source is HIRC and UART1 module clock divider as 1*/
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));

    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PA multi-function pins forUART1 TXD and RXD */

    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();

}


/*---------------------------------------------------------------------------------------------------------*/
/* Init UART1                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Reset UART1 */
    SYS_ResetModule(SYS_UART1RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
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
    /* Init UART1 */
    UART1_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+------------------------+\n");
    printf("|   IrDA function test   |\n");
    printf("+------------------------+\n");

    IrDA_FunctionTest();

    while (1);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTest(void)
{
    uint8_t u8item;

    printf("+-------------------------------------------------------------+\n");
    printf("|                     Pin Configure                           |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  ______                                      ______         |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |Master|---TXD1(PA3)  <====>  RXD1(PA2)  ---|Slave |        |\n");
    printf("| |      |                                    |      |        |\n");
    printf("| |______|                                    |______|        |\n");
    printf("|                                                             |\n");
    printf("+-------------------------------------------------------------+\n");

    /*
        UART5 is set to debug port and connect with PC firstly.
        The IrDA sample code needs two module board to execute.
        Set the master board is IrDA TX Mode and the other is IrDA Rx mode.
        Inputting char on terminal will be sent to the DEBUG_PORT of master.
        After the master receiving, the inputting char will send to DEBUG_PORT again.
        At the same time, it also sends to UART1 TX pin by IrDA mode.
        Slave will print received char after UART1 send out.
        Note that IrDA mode is ONLY used when baud rate equation is selected mode 0.

    */

    printf("\n\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     IrDA Function Test                                      |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n\n");
    u8item = getchar();

    if (u8item == '0')
        IrDA_FunctionTxTest();
    else
        IrDA_FunctionRxTest();

    printf("\nIrDA Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*                                      IrDA Function Transmit Test                                        */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest(void)
{
    uint8_t u8OutChar;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Tx Mode Test                            |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by DEBUG_PORT terminal.                    |\n");
    printf("| 2). UART1 will send a char according to step 1.           |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n\n");

    printf("\nIRDA Sample Code Start. \n");

    /* Select IrDA Tx mode and set baud rate */
    UART_SelectIrDAMode(UART1, 57600, UART_IRDA_TXEN);

    /* Wait Terminal input to send data to UART1 TX pin */
    do
    {
        u8OutChar = getchar();
        printf(" Input: %c , Send %c out\n", u8OutChar, u8OutChar);
        UART_WRITE(UART1, u8OutChar);
    } while (u8OutChar != '0');

}

/*---------------------------------------------------------------------------------------------------------*/
/*                               IrDA Function Receive Test                                            */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionRxTest(void)
{
    uint8_t u8InChar = 0xFF;

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Rx Mode Test                            |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART     |\n");
    printf("| 2). If received data is '0', the program will exit.       |\n");
    printf("|     Otherwise, print received data on terminal            |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Select IrDA Rx mode and set baud rate */
    UART_SelectIrDAMode(UART1, 57600, UART_IRDA_RXEN);

    /* Reset Rx FIFO */
    UART1->FIFO |= UART_FIFO_RXRST_Msk;

    while (UART1->FIFO & UART_FIFO_RXRST_Msk);

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do
    {
        if (UART_IS_RX_READY(UART1))
        {
            u8InChar = UART_READ(UART1);
            printf("   Input: %c \n", u8InChar);
        }
    } while (u8InChar != '0');

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

