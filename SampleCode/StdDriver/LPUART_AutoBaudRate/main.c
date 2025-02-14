/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use auto baud rate detection function.
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
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void LPUART0_Init(void);
void AutoBaudRate_Test(void);
void AutoBaudRate_TxTest(void);
void AutoBaudRate_RxTest(void);
uint32_t GetUartBaudrate(UART_T *uart);

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

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

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

    /* Set PA multi-function pins for LPUART0 TXD and RXD */
    SET_LPUART0_RXD_PA0();
    SET_LPUART0_TXD_PA1();

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
    /* Unlock protected registers */
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
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/


    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nLPUART Sample Program\n");

    /* LPUART auto baud rate sample function */
    AutoBaudRate_Test();

    printf("\nLPUART Sample Program End\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test(void)
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|-LPUART0_TXD(PA.1) <==> LPUART0_RXD(PA.0)-|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u32Item = getchar();

    if (u32Item == '0')
        AutoBaudRate_TxTest();
    else
        AutoBaudRate_RxTest();

}
/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TxTest(void)
{
    uint32_t u32Item;

    do
    {

        printf("\n");
        printf("+-----------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                 |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                   |\n");
        printf("| [2] baud rate 57600 bps                                   |\n");
        printf("| [3] baud rate 115200 bps                                  |\n");
        printf("|                                                           |\n");
        printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch (u32Item)
        {
            case '1':
                LPUART_SetLineConfig(LPUART0, 38400, LPUART_WORD_LEN_8, LPUART_PARITY_NONE, LPUART_STOP_BIT_1);
                break;

            case '2':
                LPUART_SetLineConfig(LPUART0, 57600, LPUART_WORD_LEN_8, LPUART_PARITY_NONE, LPUART_STOP_BIT_1);
                break;

            default:
                LPUART_SetLineConfig(LPUART0, 115200, LPUART_WORD_LEN_8, LPUART_PARITY_NONE, LPUART_STOP_BIT_1);
                break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        LPUART_WRITE(LPUART0, 0x1);

    } while (u32Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get LPUART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetLpUartBaudrate(LPUART_T *psLPUARTT)
{
    uint32_t u32LpUartClkSrcSel = 0ul, u32LpUartClkDivNum = 0ul;
    uint32_t u32ClkTbl[4] = {0, __LXT, __MIRC, __HIRC};
    uint32_t u32Baud_Div = 0ul;

    /* Get LPUART clock source selection and LPUART clock divider number */
    if (psLPUARTT == (LPUART_T *)LPUART0)
    {
        /* Get LPUART clock source selection */
        u32LpUartClkSrcSel = ((CLK->LPUARTSEL & CLK_LPUARTSEL_LPUART0SEL_Msk)) >> CLK_LPUARTSEL_LPUART0SEL_Pos;
        /* Get LPUART clock divider number */
        u32LpUartClkDivNum = (CLK->LPUARTDIV & CLK_LPUARTDIV_LPUART0DIV_Msk) >> CLK_LPUARTDIV_LPUART0DIV_Pos;
    }

    /* Get PCLK4 clock frequency if UART clock source selection is MIRC */
    if (u32LpUartClkSrcSel == 0ul)
    {
        u32ClkTbl[0] = CLK_GetPCLK4Freq();
    }

    /* Get LPUART baud rate divider */
    u32Baud_Div = (psLPUARTT->BAUD & LPUART_BAUD_BRD_Msk) >> LPUART_BAUD_BRD_Pos;

    /* Calculate LPUART baud rate if baud rate is set in MODE 0 */
    if ((psLPUARTT->BAUD & (LPUART_BAUD_BAUDM1_Msk | LPUART_BAUD_BAUDM0_Msk)) == LPUART_BAUD_MODE0)
        return ((u32ClkTbl[u32LpUartClkSrcSel]) / (u32LpUartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) >> 4;

    /* Calculate LPUART baud rate if baud rate is set in MODE 2 */
    else if ((psLPUARTT->BAUD & (LPUART_BAUD_BAUDM1_Msk | LPUART_BAUD_BAUDM0_Msk)) == LPUART_BAUD_MODE2)
        return ((u32ClkTbl[u32LpUartClkSrcSel]) / (u32LpUartClkDivNum + 1ul) / (u32Baud_Div + 2ul));

    /* Calculate LPUART baud rate if baud rate is set in MODE 1 */
    else if ((psLPUARTT->BAUD & (LPUART_BAUD_BAUDM1_Msk | LPUART_BAUD_BAUDM0_Msk)) == LPUART_BAUD_BAUDM1_Msk)
        return ((u32ClkTbl[u32LpUartClkSrcSel]) / (u32LpUartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) / (((psLPUARTT->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1ul);

    /* Unsupported baud rate setting */
    else
        return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_RxTest(void)
{
    /* Enable auto baud rate detect function */
    LPUART0->ALTCTL |= LPUART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while ((LPUART0->ALTCTL & LPUART_ALTCTL_ABRIF_Msk) == 0);

    if (LPUART0->FIFOSTS & LPUART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        LPUART0->FIFOSTS = LPUART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %dbps.\n", GetLpUartBaudrate(LPUART0));
    }
    else if (LPUART0->FIFOSTS & LPUART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        LPUART0->FIFOSTS = LPUART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
