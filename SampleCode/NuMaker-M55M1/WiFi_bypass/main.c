/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   A simple Wi-Fi demo for NuMaker board.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

#define WIFI_PORT       UART8        // Used to connect to Wi-Fi module
#define BYPASS_PORT     DEBUG_PORT   // Used to bypass Wi-Fi module
#define RST_PIN         PD2
#define IOCTL_INIT      \
    do{ \
        GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT); \
    }while(0)

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    CLK_EnableModuleClock(GPIOD_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable UART8 module clock */
    CLK_EnableModuleClock(UART8_MODULE);

    /* Select UART8 module clock source as HIRC and UART8 module clock divider as 1 */
    CLK_SetModuleClock(UART8_MODULE, CLK_UARTSEL1_UART8SEL_HIRC, CLK_UARTDIV1_UART8DIV(1));

    /* Reset UART8 module */
    SYS_ResetModule(SYS_UART8RST);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* For Wi-Fi module connective */
    SET_UART8_TXD_PJ0();
    SET_UART8_RXD_PJ1();
    SET_UART8_nCTS_PI14();
    SET_UART8_nRTS_PI15();

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    int32_t i;
    uint8_t au8AT_VER[] = "AT+GMR\r\n";

    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    UART_Open(WIFI_PORT, 115200);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /*
        The ESP8266 Wi-Fi module is connected to WIFI_PORT (UART8 of M55M1).
        In this demo code, BYPASS_PORT (UART0) and WIFI_PORT (UART8) are connected to
        pass through all AT commands from BYPASS_PORT to WIFI_PORT.

        User can control ESP8266 Wi-Fi module on the Terminal or by PC tool,
        e.g. "ESPlorer" (https://esp8266.ru/esplorer/)
    */

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|              ESP8266-12F Wi-Fi AT Command Bypass Sample              |\n");
    printf("+----------------------------------------------------------------------+\n");

    IOCTL_INIT;
    RST_PIN = 0;

    printf("Waiting .");
    CLK_SysTickLongDelay(3000000);

    putchar('.');
    CLK_SysTickLongDelay(1000000);
    RST_PIN = 1;

    /* Waiting for module ready */
    for (i = 0; i < 5; i++)
    {
        CLK_SysTickLongDelay(1000000);
        putchar('.');
    }

    printf(" Done\n");
    UART_RESET_RXFIFO(WIFI_PORT);
    UART_RESET_TXFIFO(WIFI_PORT);

    /* Get AT version */
    UART_Write(WIFI_PORT, au8AT_VER, sizeof(au8AT_VER));

    /* Bypass AT commands from BYPASS_PORT to WIFI_PORT */
    /* Note: A valid AT command must be end with \r\n */
    while (1)
    {
        if ((WIFI_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            while (BYPASS_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

            BYPASS_PORT->DAT = WIFI_PORT->DAT;
        }

        if ((BYPASS_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            while (WIFI_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

            WIFI_PORT->DAT = BYPASS_PORT->DAT;
        }
    }
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
