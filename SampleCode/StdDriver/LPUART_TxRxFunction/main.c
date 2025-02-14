/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Transmit and receive data from PC terminal through RS232 interface.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define RXBUFSIZE   256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void LPUART_TEST_HANDLE(void);
void LPUART_FunctionTest(void);


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

    /* Set PA multi-function pins for LPUART0 TXD and RXD and CTS and RTS */
    SET_LPUART0_RXD_PA0();
    SET_LPUART0_TXD_PA1();
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init LPUART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART0_Init()
{
    /* Reset LPUART0 */
    SYS_ResetModule(SYS_LPUART0RST);
    /* Configure LPUART0 and set LPUART0 Baudrate */
    LPUART_Open(LPUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init LPUART0 for printf and test */
    LPUART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nLPUART Sample Program\n");

    /* LPUART sample function */
    LPUART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPUART0_IRQHandler(void)
{
    LPUART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;

    if (LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_RDAINT_Msk))
    {
        printf("\nInput:");

        /* Get all the input characters */
        while (LPUART_IS_RX_READY(LPUART0))
        {
            /* Get the character from LPUART Buffer */
            u8InChar = LPUART_READ(LPUART0);

            printf("%c ", u8InChar);

            if (u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if (g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }

        printf("\nTransmission Test:");
    }

    if (LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_THREINT_Msk))
    {
        uint16_t tmp;
        tmp = g_u32comRtail;

        if (g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];

            while (LPUART_IS_TX_FULL(LPUART0)); /* Wait Tx is not full to transmit data */

            LPUART_WRITE(LPUART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }

    if (LPUART0->FIFOSTS & (LPUART_FIFOSTS_BIF_Msk | LPUART_FIFOSTS_FEF_Msk | LPUART_FIFOSTS_PEF_Msk | LPUART_FIFOSTS_RXOVIF_Msk))
    {
        LPUART_ClearIntFlag(LPUART0, (LPUART_INTSTS_RLSINT_Msk | LPUART_INTSTS_BUFERRINT_Msk));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  LPUART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect LPUART0 and PC.
        LPUART0 is set to debug port. LPUART0 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        LPUART0 will print the received char on screen.
    */

    /* Enable LPUART RDA and THRE interrupt */
    NVIC_EnableIRQ(LPUART0_IRQn);
    LPUART_EnableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_THREIEN_Msk));

    while (g_bWait);

    /* Disable LPUART RDA and THRE interrupt */
    LPUART_DisableInt(LPUART0, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_THREIEN_Msk));
    g_bWait = TRUE;
    printf("\nLPUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
