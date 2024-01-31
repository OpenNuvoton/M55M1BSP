/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief    Show how to set LPGPIO pin mode and use pin data input/output control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
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
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Enable LPGPIO module clock */
    CLK_EnableModuleClock(LPGPIO0_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Configure the GPA0 - GPA1 to LPGPIO pins.  */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA0MFP_LPIO0);
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA1MFP_Msk)) |
                    (SYS_GPA_MFP0_PA1MFP_LPIO1);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32Err, i32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|               LPGPIO Output / Input sample code               |\n");
    printf("+---------------------------------------------------------------+\n\n");
    /*------------------------------------------------------------------------------*/
    /* LPGPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin     */
    /*------------------------------------------------------------------------------*/
    printf("  >> Please connect PA.0 (LPIO0) and PA.1 (LPIO1) first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();
    LPGPIO_SetMode(LPGPIO, BIT0, LPGPIO_MODE_OUTPUT);
    LPGPIO_SetMode(LPGPIO, BIT1, LPGPIO_MODE_INPUT);
    i32Err = 0;
    printf("LPGPIO LPIO0 PA.0 (output mode) connect to LPIO1 PA.1 (input mode) ......");
    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Set LPIO0 output pin value to low */
    LPGPIO_SET_OUT_DATA(LPGPIO, (LPGPIO_GET_IN_DATA(LPGPIO) & ~BIT0));
    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for LPIO1 input pin status is low for a while */
    while ((LPGPIO_GET_IN_DATA(LPGPIO) & BIT1) != 0)
    {
        if (i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Set LPIO0 output pin value to high */
    LPGPIO_SET_OUT_DATA(LPGPIO, (LPGPIO_GET_IN_DATA(LPGPIO) | BIT0));
    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for LPIO1 input pin status is high for a while */
    while ((LPGPIO_GET_IN_DATA(LPGPIO) & BIT1) != BIT1)
    {
        if (i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Print test result */
    if (i32Err)
    {
        printf(" [FAIL].\n");
    }
    else
    {
        printf(" [OK].\n");
    }

    /* Disable LPGPIO IP clock */
    CLK_DisableModuleClock(LPGPIO0_MODULE);
    printf("Exit LPGPIO sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
