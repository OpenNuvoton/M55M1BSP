/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use M55M1 as USB Host to perform SecureISP.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_hid.h"


HID_DEV_T   *g_hid_list[CONFIG_HID_MAX_DEV];
uint8_t     s_au8BuffPool[1024] __ALIGNED(32);

extern int32_t Process_USBHCommand(HID_DEV_T *hdev);

volatile uint32_t  g_tick_cnt;
void SysTick_Handler(void)
{
    g_tick_cnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    g_tick_cnt = 0;

    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }
}

uint32_t get_ticks()
{
    return g_tick_cnt;
}

void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HXT, 0);
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Stop TIMER0 */
    TIMER_Stop(TIMER0);
    /* Clear TIMER0 interrupt flags for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    TIMER_SET_PRESCALE_VALUE(TIMER0, (12 - 1));
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* Start TIMER0 */
    TIMER_Start(TIMER0);

    while (TIMER_GetIntFlag(TIMER0) == 0);
}

void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;

    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; (i < 16) && (nBytes > 0); i++)
        {
            printf("%02x ", pucBuff[nIdx + i]);
            nBytes--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}

int  is_a_new_hid_device(HID_DEV_T *hdev)
{
    int    i;

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++)
    {
        if ((g_hid_list[i] != NULL) && (g_hid_list[i] == hdev) &&
                (g_hid_list[i]->uid == hdev->uid))
            return 0;
    }

    return 1;
}

void update_hid_device_list(HID_DEV_T *hdev)
{
    int  i = 0;
    memset(g_hid_list, 0, sizeof(g_hid_list));

    while ((i < CONFIG_HID_MAX_DEV) && (hdev != NULL))
    {
        g_hid_list[i++] = hdev;
        hdev = hdev->next;
    }
}

int  init_hid_device(HID_DEV_T *hdev)
{
    uint8_t   *pu8DataBuff;
    int32_t   i32Ret = 0;

    pu8DataBuff = (uint8_t *)((uint32_t)s_au8BuffPool);

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->idVendor, hdev->idProduct);

    i32Ret = usbh_hid_get_report_descriptor(hdev, pu8DataBuff, 1024);

    if (i32Ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        dump_buff_hex(pu8DataBuff, i32Ret);
    }

    return 0;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USB Host desired input clock is 48 MHz. Set as HIRC48 divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));

    /* Enable USB and OTG clock */
    CLK_EnableModuleClock(USBH0_MODULE);
    CLK_EnableModuleClock(USBD0_MODULE);
    CLK_EnableModuleClock(OTG0_MODULE);
    /* Enable HSUSBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);
    /* Enable CRYPTO and TRNG module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);
    CLK_EnableModuleClock(TRNG0_MODULE);
    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Set OTG as USB Host role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x1ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x1 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.8     */
    SET_USB_VBUS_EN_PB8();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.9   */
    SET_USB_VBUS_ST_PB9();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

    /* Lock protected registers */
    SYS_LockReg();
}

uint32_t CLK_GetUSBFreq(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USB Peripheral clock = HIRC48M / (USBDIV + 1) */
    return __HIRC48M / (CLK_GetModuleClockDivider(USBD0_MODULE) + 1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                        Main function                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    HID_DEV_T    *hdev, *hdev_list;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    enable_sys_tick(100);

    printf("\n\n");
    printf("System clock:   %d Hz.\n", SystemCoreClock);
    printf("USB Host clock: %d Hz.\n", CLK_GetUSBFreq());
    printf("+-----------------------------------------------+\n");
    printf("|                                               |\n");
    printf("|        USB Host Secure ISP Sample Code        |\n");
    printf("|                                               |\n");
    printf("+-----------------------------------------------+\n");
    printf("Wait until any HID devices connected...\n\n");

    usbh_core_init();
    usbh_hid_init();
    usbh_memory_used();

    memset(g_hid_list, 0, sizeof(g_hid_list));

    while (1)
    {
        if (usbh_pooling_hubs())             /* USB Host port detect polling and management */
        {
            usbh_memory_used();              /* print out USB memory allocating information */

            printf("\n Has hub events.\n");
            hdev_list = usbh_hid_get_device_list();
            hdev = hdev_list;

            while (hdev != NULL)
            {
                if (is_a_new_hid_device(hdev))
                {
                    init_hid_device(hdev);
                    Process_USBHCommand(hdev);
                }

                hdev = hdev->next;
            }

            update_hid_device_list(hdev_list);
            usbh_memory_used();
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
