/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use auto baud rate detection function
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_Test(void);
void USCI_AutoBaudRate_TxTest(void);
void USCI_AutoBaudRate_RxTest(void);
void SYS_Init(void);
void USCI0_Init(void);
uint32_t GetUuartBaudrate(UUART_T *uuart);

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

    /* Set multi-function pins for USCI0_DAT0(PA.10), USCI0_DAT1(PA.9) */
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
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

    /* USCI UART auto baud rate sample function */
    USCI_AutoBaudRate_Test();

    printf("\nUSCI UART Sample Program End\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_Test(void)
{

    uint32_t u32Item;

    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("|     Pin Configure                                          |\n");
    printf("+------------------------------------------------------------+\n");
    printf("|  ______                                             _____  |\n");
    printf("| |      |                                           |     | |\n");
    printf("| |Master|                                           |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1(PA.10) <==> USCI0_DAT0(PA.9)--|RX   | |\n");
    printf("| |______|                                           |_____| |\n");
    printf("|                                                            |\n");
    printf("+------------------------------------------------------------+\n");

    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                           |\n");
    printf("+------------------------------------------------------------+\n");
    printf("|  Description :                                             |\n");
    printf("|    The sample code needs two boards. One is Master and     |\n");
    printf("|    the other is slave. Master will send input pattern      |\n");
    printf("|    0x1 with different baud rate. It can check if Slave     |\n");
    printf("|    calculates correct baud rate.                           |\n");
    printf("+------------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                        |\n");
    printf("|  [0] Master    [1] Slave                                   |\n");
    printf("+------------------------------------------------------------+\n");
    u32Item = getchar();

    if (u32Item == '0')
        USCI_AutoBaudRate_TxTest();
    else
        USCI_AutoBaudRate_RxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_TxTest(void)
{
    uint32_t u32Item;
    uint8_t u8Char;

    do
    {

        printf("\n");
        printf("+------------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                  |\n");
        printf("+------------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                    |\n");
        printf("| [2] baud rate 57600 bps                                    |\n");
        printf("| [3] baud rate 115200 bps                                   |\n");
        printf("|                                                            |\n");
        printf("| Select baud rate and master will send 0x55 to slave ...    |\n");
        printf("+------------------------------------------------------------+\n");
        printf("| Quit                                               - [ESC] |\n");
        printf("+------------------------------------------------------------+\n\n");
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch (u32Item)
        {
            case '1':
                UUART_Open(UUART0, 38400);
                break;

            case '2':
                UUART_Open(UUART0, 57600);
                break;

            default:
                UUART_Open(UUART0, 115200);
                break;
        }

        /* Send input pattern 0x55 for auto baud rate detection */
        u8Char = 0x55;
        UUART_Write(UUART0, &u8Char, 1);

    } while (u32Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UUART Baud Rate Function                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUuartBaudrate(UUART_T *uuart)
{
    uint32_t u32PCLKFreq, u32PDSCnt, u32DSCnt, u32ClkDiv;

    /* Get PCLK frequency */
    u32PCLKFreq = CLK_GetPCLK1Freq();

    /* Get pre-divider counter */
    u32PDSCnt = ((uuart->BRGEN & UUART_BRGEN_PDSCNT_Msk) >> UUART_BRGEN_PDSCNT_Pos);

    /* Get denominator counter */
    u32DSCnt = ((uuart->BRGEN & UUART_BRGEN_DSCNT_Msk) >> UUART_BRGEN_DSCNT_Pos);

    /* Get clock divider */
    u32ClkDiv = ((uuart->BRGEN & UUART_BRGEN_CLKDIV_Msk) >> UUART_BRGEN_CLKDIV_Pos);

    return (u32PCLKFreq / (u32PDSCnt + 1) / (u32DSCnt + 1) / (u32ClkDiv + 1));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_RxTest(void)
{
    /* Set CLKDIV=DSCNT=0x5, Timing Measurement Counter Enable and Timing Measurement Counter Clock Source */
    UUART0->BRGEN = ((0x5 << UUART_BRGEN_CLKDIV_Pos) | (0x5 << UUART_BRGEN_DSCNT_Pos) | (UUART_BRGEN_TMCNTEN_Msk) | (UUART_BRGEN_TMCNTSRC_Msk));

    /* Enable auto baud rate detect function */
    UUART0->PROTCTL |= UUART_PROTCTL_ABREN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while (UUART0->PROTCTL & UUART_PROTCTL_ABREN_Msk);

    if (UUART_GET_PROT_STATUS(UUART0) & UUART_PROTSTS_ABRDETIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_ABRDETIF_Msk);
        printf("Baud rate is %dbps.\n", GetUuartBaudrate(UUART0));
    }
    else if (UUART_GET_PROT_STATUS(UUART0) & UUART_PROTSTS_ABERRSTS_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_ABERRSTS_Msk);
        printf("Error!\n");
    }

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
