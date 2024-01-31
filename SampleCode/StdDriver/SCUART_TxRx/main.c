/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate smartcard UART mode by connecting PB.4 and PB.5 pins.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
/* This is the string we used in loopback demo */
static uint8_t s_au8TxBuf[] = "Hello World!";

void SC0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
NVT_ITCM void SC0_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    while (!SCUART_GET_RX_EMPTY(SC0))
        UART_WRITE(DEBUG_PORT, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable SC0 module clock and clock source from HIRC divide 1 */
    CLK_SetModuleClock(SC0_MODULE, CLK_SCSEL_SC0SEL_HIRC, CLK_SCDIV_SC0DIV(1));

    /* Enable module clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set multi-function pins for SC UART mode */
    SET_SC0_DAT_PB4();
    SET_SC0_CLK_PB5();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    Smartcard UART Mode Sample Code     |\n");
    printf("+----------------------------------------+\n\n");
    printf("# Please connect SC0 CLK pin(PB.5) with SC0 DAT pin(PB.4) first.\n");
    printf("    - PB.5 as UART Tx\n");
    printf("    - PB.4 as UART Rx\n");
    printf("# Check UART message ... Is Hello World! ?\n\n");

    /* Open smartcard interface 0 in UART mode. The line config will be 115200-8n1 */
    /* Can call SCUART_SetLineConfig() later if necessary */
    SCUART_Open(SC0, 115200);

    /* Enable receive interrupt, no need to use other interrupts in this demo */
    SCUART_ENABLE_INT(SC0, SC_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /*
        Send the demo string out from SC0_CLK pin,
        Received data from SC0_DAT pin will be print out to UART console
    */
    SCUART_Write(SC0, s_au8TxBuf, sizeof(s_au8TxBuf));

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
