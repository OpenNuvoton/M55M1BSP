/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Simulate an USB HID mouse and HID keyboard.
 *           Mouse draws circle on the screen and
 *           Keyboard use GPIO to simulate key input.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_mousekeyboard.h"

#define CRYSTAL_LESS        0
#define TRIM_INIT           (SYS_BASE+0xF40)

void SetDebugUartCLK(void)
{
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)

#if (CRYSTAL_LESS)

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(DEBUG_PORT_MODULE, CLK_UARTSEL0_UART6SEL_HIRC, CLK_UARTDIV0_UART6DIV(1));

#else

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(DEBUG_PORT_MODULE, CLK_UARTSEL0_UART6SEL_HXT, CLK_UARTDIV0_UART6DIV(1));

#endif

    /* Enable UART clock */
    CLK_EnableModuleClock(DEBUG_PORT_MODULE);

    /* Reset UART module */
    SYS_ResetModule(DEBUG_PORT_RST);

#endif /* !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING) */
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

#if (!CRYSTAL_LESS)

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);

    /* Select USB clock source as APLL1/2 and USB clock divider as 2 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));

#else

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ);

    /* Enable HIRC48M clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Waiting for HIRC48M clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));

#endif

    /* Enable all GPIO module clock */
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

    /* Enable OTG0_ module clock */
    CLK_EnableModuleClock(OTG0_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();
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
}

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    PMC_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (PMC->PWRCTL & PMC_PWRCTL_PDEN_Msk)
        PMC->PWRCTL ^= PMC_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    /* Init BTN0 */
    GPIO_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|     NuMicro USB HID Mouse and Keyboard Sample Code     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("Mouse draws circle on the screen.\n");
    printf("If press BTN0 button, just report it is key 'a'.\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while (1)
    {
#if CRYSTAL_LESS

        /* Start USB trim if it is not enabled. */
        if ((SYS->TCTL48M & SYS_TCTL48M_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /* Re-enable crystal-less */
                SYS->TCTL48M = 0x01;
                SYS->TCTL48M |= SYS_TCTL48M_REFCKSEL_Msk | SYS_TCTL48M_BOUNDEN_Msk | (8 << SYS_TCTL48M_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if (SYS->TISTS48M & (SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->TISTS48M = 0;

            /* Clear error flags */
            SYS->TISTS12M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        /* Enter power down when USB suspend */
        if (g_u8Suspend)
            PowerDown();

        HID_UpdateMouseData();
        HID_UpdateKbData();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
