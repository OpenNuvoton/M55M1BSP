/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to light up the WS1812B LED array.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "WS1812B_driver_LED.h"

#define LED_NUMBER  8
#define PIN_NUMBER  2

typedef enum
{
    eCASE_GREEN_BLUE = 0,
    eCASE_RED_GREEN,
    eCASE_BLUE_RED,
    eCASE_WHITE,
} E_LED_COLOR;

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO0_MODULE);

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 1 */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_HIRC, CLK_PSIODIV_PSIO0DIV(1));

    CLK_EnableModuleClock(GPIOE_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PSIO multi-function pin CH0(PE.14) and CH1(PE.15) */
    SET_PSIO0_CH0_PE14();
    SET_PSIO0_CH1_PE15();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* For library internal used, this memory size should be (data length)*3 word at least, */
    /* this case is  LED_NUMBER*PIN_NUMBER*3 word.                                          */
    uint32_t   au32LedPattern[LED_NUMBER * PIN_NUMBER], au32InternalUse[LED_NUMBER * PIN_NUMBER * 3];

    S_PSIO_WS2812B_LED_CFG  sConfig;
    E_LED_COLOR             eColor = eCASE_GREEN_BLUE;
    WS2812B_LED_Pin_CFG     sPinCFG = {PSIO_PIN0, PSIO_PIN1, 0, 0, 0, 0, 0, 0}; //Enable Pin0 and Pin1

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|         Worldsemi WS2812B LED sample code            | \n");
    printf("|          Please connected PSIO_CH0(PE.14)            | \n");
    printf("|           and PSIO_CH1(PE.15) to device              | \n");
    printf("+------------------------------------------------------+ \n");

    /* Set Led configuration */
    sConfig.u8SlotCtrl      = PSIO_SC0;
    sConfig.u8PDMAChannel   = 0;
    sConfig.pu8PinCFG       = sPinCFG;
    sConfig.u8PinNumber     = 2;
    sConfig.pu32DataAddr    = au32LedPattern;
    sConfig.u32DataLength   = LED_NUMBER * PIN_NUMBER;
    sConfig.pu32InternalMemory = au32InternalUse;           /* For library internal used, the memory size should be (data length)*3 word at least. */
    /* This case is  LED_NUMBER*PIN_NUMBER*3 word. */

    /* Initialize PSIO setting for WS2812B LED */
    PSIO_WS2812B_Open(&sConfig);

    do
    {
        switch (eColor)
        {
            case eCASE_GREEN_BLUE:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_GREEN;
                au32LedPattern[1]    = WS2812B_BLUE;    //LED0
                au32LedPattern[2]    = WS2812B_GREEN;
                au32LedPattern[3]    = WS2812B_BLUE;    //LED1
                au32LedPattern[4]    = WS2812B_GREEN;
                au32LedPattern[5]    = WS2812B_BLUE;    //LED2
                au32LedPattern[6]    = WS2812B_GREEN;
                au32LedPattern[7]    = WS2812B_BLUE;    //LED3
                au32LedPattern[8]    = WS2812B_GREEN;
                au32LedPattern[9]    = WS2812B_BLUE;    //LED4
                au32LedPattern[10]   = WS2812B_GREEN;
                au32LedPattern[11]   = WS2812B_BLUE;    //LED5
                au32LedPattern[12]   = WS2812B_GREEN;
                au32LedPattern[13]   = WS2812B_BLUE;    //LED6
                au32LedPattern[14]   = WS2812B_GREEN;
                au32LedPattern[15]   = WS2812B_BLUE;    //LED7
                eColor = eCASE_RED_GREEN;
                break;

            case eCASE_RED_GREEN:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_RED;
                au32LedPattern[1]    = WS2812B_GREEN;   //LED0
                au32LedPattern[2]    = WS2812B_RED;
                au32LedPattern[3]    = WS2812B_GREEN;   //LED1
                au32LedPattern[4]    = WS2812B_RED;
                au32LedPattern[5]    = WS2812B_GREEN;   //LED2
                au32LedPattern[6]    = WS2812B_RED;
                au32LedPattern[7]    = WS2812B_GREEN;   //LED3
                au32LedPattern[8]    = WS2812B_RED;
                au32LedPattern[9]    = WS2812B_GREEN;   //LED4
                au32LedPattern[10]   = WS2812B_RED;
                au32LedPattern[11]   = WS2812B_GREEN;   //LED5
                au32LedPattern[12]   = WS2812B_RED;
                au32LedPattern[13]   = WS2812B_GREEN;   //LED6
                au32LedPattern[14]   = WS2812B_RED;
                au32LedPattern[15]   = WS2812B_GREEN;   //LED7
                eColor = eCASE_BLUE_RED;
                break;

            case eCASE_BLUE_RED:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_BLUE;
                au32LedPattern[1]    = WS2812B_RED;     //LED0
                au32LedPattern[2]    = WS2812B_BLUE;
                au32LedPattern[3]    = WS2812B_RED;     //LED1
                au32LedPattern[4]    = WS2812B_BLUE;
                au32LedPattern[5]    = WS2812B_RED;     //LED2
                au32LedPattern[6]    = WS2812B_BLUE;
                au32LedPattern[7]    = WS2812B_RED;     //LED3
                au32LedPattern[8]    = WS2812B_BLUE;
                au32LedPattern[9]    = WS2812B_RED;     //LED4
                au32LedPattern[10]   = WS2812B_BLUE;
                au32LedPattern[11]   = WS2812B_RED;     //LED5
                au32LedPattern[12]   = WS2812B_BLUE;
                au32LedPattern[13]   = WS2812B_RED;     //LED6
                au32LedPattern[14]   = WS2812B_BLUE;
                au32LedPattern[15]   = WS2812B_RED;     //LED7
                eColor = eCASE_WHITE;
                break;

            case eCASE_WHITE:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_WHITE;
                au32LedPattern[1]    = WS2812B_WHITE;   //LED0
                au32LedPattern[2]    = WS2812B_WHITE;
                au32LedPattern[3]    = WS2812B_WHITE;   //LED1
                au32LedPattern[4]    = WS2812B_WHITE;
                au32LedPattern[5]    = WS2812B_WHITE;   //LED2
                au32LedPattern[6]    = WS2812B_WHITE;
                au32LedPattern[7]    = WS2812B_WHITE;   //LED3
                au32LedPattern[8]    = WS2812B_WHITE;
                au32LedPattern[9]    = WS2812B_WHITE;   //LED4
                au32LedPattern[10]   = WS2812B_WHITE;
                au32LedPattern[11]   = WS2812B_WHITE;   //LED5
                au32LedPattern[12]   = WS2812B_WHITE;
                au32LedPattern[13]   = WS2812B_WHITE;   //LED6
                au32LedPattern[14]   = WS2812B_WHITE;
                au32LedPattern[15]   = WS2812B_WHITE;   //LED7
                eColor = eCASE_GREEN_BLUE;
                break;
        }

        /* Send LED pattern by PSIO */
        PSIO_WS2812B_Send_Pattern(&sConfig);

        /* Delay 500 ms */
        CLK_SysTickLongDelay(500000);
    } while (1);

}
