/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Use embedded flash as storage to implement a USB Mass-Storage device.
 *
 * @note     Windows may prompt "There's a problem with this drive" upon connection.
 *           This is harmless. For code size constraints, this ISP implements a minimal
 *           "Fake FAT" that ignores Windows' background metadata writes.
 *           Please safely ignore the warning and drag-and-drop the .bin file directly.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "MassStorage.h"

#define CRYSTAL_LESS 1

int32_t SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
#if (!CRYSTAL_LESS)
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select SCLK to HIRC before APLL setting*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    /* Enable APLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ, CLK_APLL0_SELECT);
    /* Set clock with limitations */
    CLK_SET_HCLK2DIV(2);
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Switch SCLK clock source to APLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);
    /* Select USB clock source as APLL1/2 and USB clock divider as 1 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));
#else
    /* Select SCLK to HIRC before APLL setting*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    /* Enable APLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);
    /* Set clock with limitations */
    CLK_SET_HCLK2DIV(2);
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Switch SCLK clock source to APLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);
    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif

    /* Enable module clock */
    CLK_EnableModuleClock(ISP0_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    /* Enable OTG0 module clock */
    CLK_EnableModuleClock(OTG0_MODULE);
    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set DETECT_PIN to input mode */
    PI->MODE &= ~(GPIO_MODE_MODE11_Msk);
    SET_GPIO_PI11();

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock write-protected registers */
    SYS_UnlockReg();
    /* Enable ISP */
    FMC_Open();

    /* Init System, peripheral clock and multi-function I/O */
    /* Check if DETECT_PIN is low to enter ISP flow */
    if ((SYS_Init() < 0) || (DETECT_PIN != 0))
        goto _APROM;

    /* Enable APROM update */
    FMC_ENABLE_AP_UPDATE();

    /* Endpoint configuration */
    MSC_Init();

    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim value */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

    while (1)
    {

#if CRYSTAL_LESS

        /* Start USB trim function if it is not enabled. */
        if ((SYS->TCTL48M & SYS_TCTL48M_FREQSEL_Msk) != 0x1)
        {
            /* Start USB trim only when USB signal arrived */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /*
                    USB clock trim function:
                    HIRC Trimming with boundary function enhances robustility
                    and keeps HIRC in right frequency while receiving unstable USB signal
                */
                SYS->TCTL48M = (0x1 << SYS_TCTL48M_FREQSEL_Pos) | SYS_TCTL48M_REFCKSEL_Msk |
                               SYS_TCTL48M_BOUNDEN_Msk | (8 << SYS_TCTL48M_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when any error found */
        if (SYS->TISTS48M & (SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable USB clock trim function */
            SYS->TCTL48M = 0;

            /* Clear trim error flags */
            SYS->TCTL48M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        MSC_ProcessCmd();
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
