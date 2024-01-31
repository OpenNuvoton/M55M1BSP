/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through I2C interface.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip I2C and assign update file
 *           of Flash.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
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
    CyclesPerUs     = SystemCoreClock / 1000000UL;

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Check clock stable */
    u32TimeOutCnt = SystemCoreClock >> 1;

    while ((CLK->UARTCTL & BIT31) == 0UL)
    {
        if (--u32TimeOutCnt == 0)
        {
            return -1;
        }
    }

    /* Enable I2C1 clock */
    CLK->I2CCTL |= CLK_I2CCTL_I2C1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set I2C1 multi-function pins */
    SET_I2C1_SDA_PA2();
    SET_I2C1_SCL_PA3();

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;

    return 0;
}

int main(void)
{
    uint32_t au32CmdBuff[16], u32TimeoutInMS = 300;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Enable ISP */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    g_u32ApromSize = GetApromSize();

    I2C_Init();

    while (u32TimeoutInMS > 0)
    {
        if (bI2cDataReady == 1)
        {
            goto _ISP;
        }

        CLK_SysTickDelay(1000);
        u32TimeoutInMS--;
    }

    /* Timeout then go to APROM */
    if (u32TimeoutInMS == 0)
        goto _APROM;

_ISP:

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            memcpy(au32CmdBuff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)au32CmdBuff, 64);
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
