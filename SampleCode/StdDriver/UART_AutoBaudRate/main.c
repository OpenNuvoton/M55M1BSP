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
void UART1_Init(void);
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

    /* Set PA multi-function pins for UART1 TXD and RXD */
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
    /* Init UART1 */
    UART1_Init();

    /* Lock protected registers */
    SYS_LockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/


    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART auto baud rate sample function */
    AutoBaudRate_Test();

    printf("\nUART Sample Program End\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                            */
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
    printf("| |Master|--UART1_TXD(PA.3) <====> UART1_RXD(PA.2)--|Slave| |\n");
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
                UART_SetLineConfig(UART1, 38400, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
                break;

            case '2':
                UART_SetLineConfig(UART1, 57600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
                break;

            default:
                UART_SetLineConfig(UART1, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
                break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART_WRITE(UART1, 0x1);

    } while (u32Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUartBaudrate(UART_T *psUART)
{
    uint8_t u32UartClkSrcSel = 0, u32UartClkDivNum = 0;
    uint32_t u32ClkTbl[5] = {__HXT, __HIRC, __LXT, 0, __HIRC48M};
    uint32_t u32Baud_Div;

    if (psUART == UART0)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART0SEL_Msk) >> CLK_UARTSEL0_UART0SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART0DIV_Msk) >> CLK_UARTDIV0_UART0DIV_Pos;
    }
    else if (psUART == UART1)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART1SEL_Msk) >> CLK_UARTSEL0_UART1SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART1DIV_Msk) >> CLK_UARTDIV0_UART1DIV_Pos;
    }
    else if (psUART == UART2)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART2SEL_Msk) >> CLK_UARTSEL0_UART2SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART2DIV_Msk) >> CLK_UARTDIV0_UART2DIV_Pos;
    }
    else if (psUART == (UART_T *)UART3)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART3SEL_Msk) >> CLK_UARTSEL0_UART3SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART3DIV_Msk) >> CLK_UARTDIV0_UART3DIV_Pos;
    }
    else if (psUART == (UART_T *)UART4)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART4SEL_Msk) >> CLK_UARTSEL0_UART4SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART4DIV_Msk) >> CLK_UARTDIV0_UART4DIV_Pos;
    }
    else if (psUART == (UART_T *)UART5)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART5SEL_Msk) >> CLK_UARTSEL0_UART5SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART5DIV_Msk) >> CLK_UARTDIV0_UART5DIV_Pos;
    }
    else if (psUART == (UART_T *)UART6)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART6SEL_Msk) >> CLK_UARTSEL0_UART6SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART6DIV_Msk) >> CLK_UARTDIV0_UART6DIV_Pos;
    }
    else if (psUART == (UART_T *)UART7)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL0_UART7SEL_Msk) >> CLK_UARTSEL0_UART7SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV0_UART7DIV_Msk) >> CLK_UARTDIV0_UART7DIV_Pos;
    }
    else if (psUART == (UART_T *)UART8)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL1 & CLK_UARTSEL1_UART8SEL_Msk) >> CLK_UARTSEL1_UART8SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV1 & CLK_UARTDIV1_UART8DIV_Msk) >> CLK_UARTDIV1_UART8DIV_Pos;
    }
    else if (psUART == (UART_T *)UART9)
    {
        /* Get UART clock source selection */
        u32UartClkSrcSel = (CLK->UARTSEL0 & CLK_UARTSEL1_UART9SEL_Msk) >> CLK_UARTSEL1_UART9SEL_Pos;
        /* Get UART clock divider number */
        u32UartClkDivNum = (CLK->UARTDIV0 & CLK_UARTDIV1_UART9DIV_Msk) >> CLK_UARTDIV1_UART9DIV_Pos;
    }

    /* Get PLL clock frequency if UART clock source selection is PLL/2 */
    if (u32UartClkSrcSel == 3ul)
    {
        u32ClkTbl[u32UartClkSrcSel] = CLK_GetAPLL0ClockFreq() / 2;
    }

    /* Get UART baud rate divider */
    u32Baud_Div = (psUART->BAUD & UART_BAUD_BRD_Msk) >> UART_BAUD_BRD_Pos;

    /* Calculate UART baud rate if baud rate is set in MODE 0 */
    if ((psUART->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE0)
        return ((u32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) >> 4;

    /* Calculate UART baud rate if baud rate is set in MODE 2 */
    else if ((psUART->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE2)
        return ((u32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul) / (u32Baud_Div + 2ul));

    /* Calculate UART baud rate if baud rate is set in MODE 1 */
    else if ((psUART->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_BAUDM1_Msk)
        return ((u32ClkTbl[u32UartClkSrcSel]) / (u32UartClkDivNum + 1ul) / (u32Baud_Div + 2ul)) / (((psUART->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1ul);

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
    UART1->ALTCTL |= UART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while ((UART1->ALTCTL & UART_ALTCTL_ABRIF_Msk) == 0);

    if (UART1->FIFOSTS & UART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %dbps.\n", GetUartBaudrate(UART1));
    }
    else if (UART1->FIFOSTS & UART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}




/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
