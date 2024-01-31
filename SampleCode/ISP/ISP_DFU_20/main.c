/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to upgrade firmware between HUSB device and PC
 *           through USB DFU (Device Firmware Upgrade) class.
 *           WindowsDriver and WindowsTool are also included in this sample code
 *           to connect with USB device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "fmc_user.h"
#include "dfu_transfer.h"

#define PLL_CLOCK       FREQ_180MHZ

int32_t  g_FMC_i32ErrCode = 0;
uint32_t g_u32ApromSize;

// Empty function to reduce code size
uint32_t ProcessHardFault(uint32_t *pu32StackFrame)
{
    return 0;
}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1; /* 500ms time-out */

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz and HXT clock */
    CLK->SRCCTL |= (CLK_SRCCTL_HIRCEN_Msk | CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    while ((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk)) != (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk))
    {
        if (--u32TimeOutCnt == 0)
        {
            return -1;
        }
    }

    /* Select SCLK to HIRC before APLL setting*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    /* Enable APLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Set clock with limitations */
    CLK_SET_HCLK2DIV(2);
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Switch SCLK clock source to APLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select HSUSBD */
    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;

    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSOTGPHYEN_Msk;
    CLK_SysTickDelay(20);   // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable module clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB.12 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE12_Msk);
    SET_GPIO_PB12();

    return 0;
}

void HSUSBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    if ((SYS_Init() < 0) || (DETECT_PIN != 0))
        goto _APROM;

    /* Enable ISP */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    /* Open HSUSBD controller */
    HSUSBD_Open(NULL, DFU_ClassRequest, NULL);

    /*Init Endpoint configuration for DFU */
    DFU_Init();

    /* Start transaction */
    HSUSBD->OPER = HSUSBD_OPER_HISPDEN_Msk;   /* high-speed */
    HSUSBD_CLR_SE0();

    /* M55M1 has 8 KB LDROM, changed to use IRQ mode */
    /* Enable HSUSBD interrupt */
    //NVIC_EnableIRQ(HSUSBD_IRQn);

    while (DETECT_PIN == 0)
    {
        HSUSBD_IRQHandler();
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
