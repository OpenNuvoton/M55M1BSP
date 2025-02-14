/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Simulate an USB mouse and draws circle on the screen.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_mouse.h"

static uint8_t volatile s_u8RemouteWakeup = 0;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);

    /* Select HSOTG PHY Reference clock frequency which is from HXT */
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

    /* Set HSUSB role to HSUSBD */
    SET_HSUSBDROLE();

    /* Enable HSUSB PHY */
    SYS_Enable_HSUSB_PHY();

    /* Enable HSUSBD peripheral clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();
}

void GPIO_Init(void)
{
    // GPI.11 Input for button. Active low.
    SET_GPIO_PI11();
    /* Enable PI11 interrupt for wakeup */
    GPIO_SetMode(PI, BIT11, GPIO_MODE_QUASI);
    GPIO_EnableInt(PI, 11, GPIO_INT_FALLING);
    PI->DBEN |= BIT11;            // eanble debounce
    PI->DBCTL = GPIO_DBCTL_DBCLKSEL_32768;  // Debounce time is about 3.6 ms

    NVIC_EnableIRQ(GPI_IRQn);
}

/* GPI Interrupt handler */
NVT_ITCM void GPI_IRQHandler(void)
{
    /* Clear PI11 interrupt flag */
    GPIO_CLR_INT_FLAG(PI, BIT11);
    s_u8RemouteWakeup = 1;
}

void PowerDown(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    HSUSBD->PHYCTL |= HSUSBD_PHYCTL_VBUSWKEN_Msk | HSUSBD_PHYCTL_LINESTATEWKEN_Msk;

    PMC_PowerDown();

    g_u8Suspend = 0;
    HSUSBD_ENABLE_USB();
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!(HSUSBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
        if (--u32TimeOutCnt == 0) break;


    /* Note HOST to resume USB tree if it is suspended and remote wakeup enabled */
    if (g_hsusbd_RemoteWakeupEn && s_u8RemouteWakeup)
    {
        /* Generate resume */
        HSUSBD->OPER |= HSUSBD_OPER_RESUMEEN_Msk;
        s_u8RemouteWakeup = 0;
    }

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    GPIO_Init();

    printf("NuMicro HSUSBD HID\n");

    HSUSBD_Open(&gsHSInfo, HID_ClassRequest, NULL);

#ifdef SUPPORT_LPM
    HSUSBD_ENABLE_LPM();
#endif

    /* Endpoint configuration */
    HID_Init();

    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(HSUSBD_IRQn);

    /* Start transaction */
    HSUSBD_Start();

    while (1)
    {
        /* Enter power down when USB suspend */
        if (g_u8Suspend)
        {
            PowerDown();

            /* Waiting for key release */
            while ((GPIO_GET_IN_DATA(PI) & BIT11) != BIT11);
        }

        /* Move mouse when Key pressed */
        if ((GPIO_GET_IN_DATA(PI) & BIT11) == 0x0)
            HID_UpdateMouseData();
    }
}
