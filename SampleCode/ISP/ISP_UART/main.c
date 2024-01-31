/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through UART interface
 *           between chip UART and PC UART.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip UART and assign update file
 *           of Flash.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "uart_transfer.h"

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1; /* 500ms time-out */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable external high speed crystal clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    /* Waiting for external high speed crystal clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
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
    PllClock        = PLL_CLOCK;
    SystemCoreClock = PllClock;
    CyclesPerUs = SystemCoreClock / 1000000UL;

    /* Enable module clock */
    CLK->FMCCTL |= CLK_FMCCTL_ISP0CKEN_Msk;
    CLK->UARTSEL0 = (CLK->UARTSEL0 & ~CLK_UARTSEL0_UART6SEL_Msk) | CLK_UARTSEL0_UART6SEL_HXT;
    CLK->UARTCTL |= CLK_UARTCTL_UART6CKEN_Msk;
    /* Check clock stable */
    u32TimeOutCnt = SystemCoreClock >> 1;

    while ((CLK->UARTCTL & BIT31) == 0UL)
    {
        if (--u32TimeOutCnt == 0)
        {
            return -1;
        }
    }

    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART6 RXD and TXD */
    SET_UART6_RXD_PH5();
    SET_UART6_TXD_PH4();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeoutInMS = 300;

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init UART */
    UART_Init();

    /* Enable ISP */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (u32TimeoutInMS > 0)
    {
        /* Wait for CMD_CONNECT command */
        if ((g_u8bufhead >= 4) || (g_u8bUartDataReady == TRUE))
        {
            uint32_t u32lcmd;
            u32lcmd = inpw((uint32_t)g_au8uart_rcvbuf);

            if (u32lcmd == CMD_CONNECT)
            {
                goto _ISP;
            }
            else
            {
                g_u8bUartDataReady = FALSE;
                g_u8bufhead = 0;
            }
        }

        CLK_SysTickDelay(1000);
        u32TimeoutInMS--;
    }

    /* Timeout then go to APROM */
    if (u32TimeoutInMS == 0)
        goto _APROM;

_ISP:

    /* Parse command from master and send response back */
    while (1)
    {
        if (g_u8bUartDataReady == TRUE)
        {
            g_u8bUartDataReady = FALSE;         /* Reset UART data ready flag */
            ParseCmd(g_au8uart_rcvbuf, 64);     /* Parse command from master */
            PutString();                        /* Send response to master */
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
