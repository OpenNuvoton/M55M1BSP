/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_ED25519 code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "EdDsa.h"

#define TXT_MAX_LEN     8192

volatile uint32_t  g_tick_cnt;

extern int ed25519_test();

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


    /* Enable PLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}

int main()
{
    SYS_UnlockReg();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    NVIC_EnableIRQ(CRYPTO_IRQn);

    SHA_ENABLE_INT(CRYPTO);
    ECC_ENABLE_INT(CRYPTO);
    CRYPTO->PAP_CTL |= CRYPTO_PAP_CTL_PAPEN_Msk;

    printf("+------------------------------------------+\n");
    printf("|   Start ED25519 Example                  |\n");
    printf("+------------------------------------------+\n");


    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }

    printf("\n\nStart ed25519  Sig and Verify!\n");

    ed25519_test();

    printf("\n\ned25519  Sig and Verify Pass!\n");

    /* Got no where to go, just loop forever */
    while (1);
}


void SysTick_Handler(void)
{
    g_tick_cnt++;
    //SendChar_ToUART('#');
}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

