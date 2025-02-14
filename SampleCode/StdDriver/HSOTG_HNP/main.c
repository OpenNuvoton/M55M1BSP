/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample demonstrates OTG HNP.
 *           This sample needs two M55 boards. One is as Host, the other is as Device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "hub.h"
#include "usbh_lib.h"
#include "usbh_hid.h"
#include "hid_mouse.h"

#ifdef __ICCARM__
    #pragma data_alignment=32
    uint8_t  g_buff_pool[1024];
#else
    uint8_t   g_buff_pool[1024] __attribute__((aligned(32)));
#endif

HID_DEV_T   *g_hid_list[CONFIG_HID_MAX_DEV];
UDEV_T *gOTG_Dev_pet;

uint8_t volatile gStartHNP = 0;
uint8_t volatile otg_role_change = 0;
uint32_t volatile intcount = 0;
volatile uint32_t  g_tick_cnt;

NVT_ITCM void SysTick_Handler(void)
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

/*
 *  USB device connect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  HSUSBH port, USBH port, and USB hub device ports. Once a new device connected, it
 *  will be detected and enumerated in the call to usbh_pooling_hubs(). This callback
 *  will be invoked from USB core once a newly connected device was successfully enumerated.
 */
void  connect_func(struct udev_t *udev, int param)
{
    struct hub_dev_t *parent;
    int    i;

    gOTG_Dev_pet = udev;
    parent = udev->parent;

    printf("Device [0x%x,0x%x] was connected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
    printf("    Speed:    %s-speed\n", (udev->speed == SPEED_HIGH) ? "high" : ((udev->speed == SPEED_FULL) ? "full" : "low"));
    printf("    Location: ");

    if (parent == NULL)
    {
        if (udev->port_num == 1)
            printf("USB 2.0 port\n");
        else
            printf("USB 1.1 port\n");
    }
    else
    {
        if (parent->pos_id[0] == '1')
            printf("USB 2.0 port");
        else
            printf("USB 1.1 port");

        for (i = 1; parent->pos_id[i] != 0; i++)
        {
            printf(" => Hub port %c", parent->pos_id[i]);
        }

        printf(" => Hub port %d\n", udev->port_num);

        printf("\n");
    }

    printf("\n");
}


/*
 *  USB device disconnect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  HSUSBH port, USBH port, and USB hub device ports. Once a device was disconnected, it
 *  will be detected and removed in the call to usbh_pooling_hubs(). This callback
 *  will be invoked from USB core prior to remove that device.
 */
void  disconnect_func(struct udev_t *udev, int param)
{
    printf("Device [0x%x,0x%x] was disconnected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from HIRC_12M. Prescale 12
     */
    /* TIMER0 clock from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);

    TIMER_SET_PRESCALE_VALUE(TIMER0, (12 - 1));
    /* stop timer0 */
    TIMER_Stop(TIMER0);
    /* write 1 to clear for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* set timer cmp value */
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    /* Timer0 config to oneshot mode */
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* start timer0*/
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

void  int_read_callback(HID_DEV_T *hdev, uint16_t ep_addr, int status, uint8_t *rdata, uint32_t data_len)
{
    /*
     *  USB host HID driver notify user the transfer status via <status> parameter. If the
     *  If <status> is 0, the USB transfer is fine. If <status> is not zero, this interrupt in
     *  transfer failed and HID driver will stop this pipe. It can be caused by USB transfer error
     *  or device disconnected.
     */
    if (status < 0)
    {
        printf("Interrupt in transfer failed! status: %d\n", status);
        return;
    }

    printf("Device [0x%x,0x%x] ep 0x%x, %d bytes received =>\n",
           hdev->idVendor, hdev->idProduct, ep_addr, data_len);
    dump_buff_hex(rdata, data_len);

    intcount++;
}

int  init_hid_device(HID_DEV_T *hdev)
{
    uint8_t   *data_buff;
    int       i, ret;

    data_buff = (uint8_t *)((uint32_t)g_buff_pool);

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->idVendor, hdev->idProduct);

    ret = usbh_hid_get_report_descriptor(hdev, data_buff, 1024);

    if (ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        dump_buff_hex(data_buff, ret);
    }

    /*
     *  Example: GET_PROTOCOL request.
     */
    ret = usbh_hid_get_protocol(hdev, data_buff);
    //printf("[GET_PROTOCOL] ret = %d, protocol = %d\n", ret, data_buff[0]);

    /*
     *  Example: SET_PROTOCOL request.
     */
    ret = usbh_hid_set_protocol(hdev, data_buff[0]);
    //printf("[SET_PROTOCOL] ret = %d, protocol = %d\n", ret, data_buff[0]);

    /*
     *  Example: GET_REPORT request on report ID 0x1, report type FEATURE.
     */
    ret = usbh_hid_get_report(hdev, RT_FEATURE, 0x1, data_buff, 64);

    if (ret > 0)
    {
        printf("[GET_REPORT] Data => ");

        for (i = 0; i < ret; i++)
            printf("%02x ", data_buff[i]);

        printf("\n");
    }

    //gStartHNP = 1;

    printf("\nUSBH_HidStartIntReadPipe...\n");
    ret = usbh_hid_start_int_read(hdev, 0, int_read_callback);

    if (ret != HID_RET_OK)
        printf("usbh_hid_start_int_read failed! %d\n", ret);
    else
        printf("Interrupt in transfer started...\n");

    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Enable GPIOA module clock */
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

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT*/
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

    /* Enable USBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);

    /* Enable HSUSBD module clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Set OTG as USB OTG role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x3ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x3 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;//Set HSUSB PHY Active.


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();


    SystemCoreClockUpdate();
    /* Lock protected registers */
    SYS_LockReg();

}

void OTG_SetFeature(uint32_t value)
{
    uint32_t  read_len;
    /* 0x3 - b_hnp_enable, 0x4 - a_hnp_support */
    /* set feature */
    usbh_ctrl_xfer(gOTG_Dev_pet, REQ_TYPE_OUT | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                   USB_REQ_SET_FEATURE, value, 0, 0,
                   NULL, &read_len, 300);
}

/**
  * @brief  HSUSBOTG_IRQHandler, HSOTG interrupt handler.
  * @param  None.
  * @retval None.
  */
NVT_ITCM void HSOTG_IRQHandler(void)
{
    __IO uint32_t reg, en, i;

    en = HSOTG->INTEN;
    reg = HSOTG->INTSTS;

    if (reg & en & HSOTG_INTSTS_ROLECHGIF_Msk)
    {
        printf("[Role Change]\n");
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_ROLECHGIF_Msk);
    }

    if (reg & en & HSOTG_INTSTS_GOIDLEIF_Msk)
    {
        printf("[Go Idle] 0x%x\n", HSOTG->STATUS);
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_GOIDLEIF_Msk);
        otg_role_change = 0;
    }

    if (reg & en & HSOTG_INTSTS_IDCHGIF_Msk)
    {
        printf("[ID change 0x%x]\n", HSOTG->STATUS);
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_IDCHGIF_Msk);

        if ((HSOTG->STATUS & HSOTG_STATUS_IDSTS_Msk) == 0)
            HSOTG->CTL |= HSOTG_CTL_BUSREQ_Msk;
    }

    if (reg & en & HSOTG_INTSTS_VBEIF_Msk)
    {
        printf("[VBUS Err]\n");
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_VBEIF_Msk);
    }

    if (reg & en & HSOTG_INTSTS_SRPFIF_Msk)
    {
        printf("[SRP Fail]\n");
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_SRPFIF_Msk);
    }

    if (reg & en & HSOTG_INTSTS_HNPFIF_Msk)
    {
        printf("[HNP Fail]\n");
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_HNPFIF_Msk);
        otg_role_change = 0;
    }

    if (reg & en & HSOTG_INTSTS_SRPDETIF_Msk)
    {
        printf("[SRP Detect]\n");
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_SRPDETIF_Msk);

        for (i = 0; i < 10000; i++);

        HSOTG->CTL |= HSOTG_CTL_BUSREQ_Msk;
    }

    if (reg & en & HSOTG_INTSTS_HOSTIF_Msk)
    {
        printf("[Act as Host]\n");
        HSOTG_DISABLE_INT(HSOTG_INTEN_HOSTIEN_Msk);
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_HOSTIF_Msk);

        if ((HSOTG->STATUS & 0x80) | (HSOTG->STATUS & 0x10))
        {
            if (otg_role_change == 3)
                otg_role_change = 4;
            else
                otg_role_change = 2;
        }
    }

    if (reg & en & HSOTG_INTSTS_PDEVIF_Msk)
    {
        printf("[Act as Peripheral]\n");
        HSOTG_DISABLE_INT(HSOTG_INTEN_PDEVIEN_Msk);
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_PDEVIF_Msk);

        if (HSOTG->STATUS & 0x40)
        {
            if (otg_role_change == 2)
                otg_role_change = 4;
            else
                otg_role_change = 3;
        }
    }

    /* make sure that interrupt flag has been cleared. */
    reg = HSOTG->INTSTS;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    HID_DEV_T    *hdev, *hdev_list;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    enable_sys_tick(100);

    printf("\n\n");
    printf("system clock :%uHz\r\n", SystemCoreClock);
    printf("+------------------------------+\n");
    printf("|                              |\n");
    printf("|     USB HSOTG HNP sample     |\n");
    printf("|                              |\n");
    printf("+------------------------------+\n");

    HSOTG_ENABLE_PHY();
    HSOTG_ENABLE_ID_DETECT();
    NVIC_EnableIRQ(HSOTG_IRQn);
    delay_us(1000);

    /* clear interrupt and enable relative interrupts */
    HSOTG_ENABLE_INT(HSOTG_INTEN_VBEIEN_Msk | HSOTG_INTEN_SRPFIEN_Msk | HSOTG_INTEN_HNPFIEN_Msk | HSOTG_INTEN_GOIDLEIEN_Msk |
                     HSOTG_INTEN_SRPDETIEN_Msk | HSOTG_INTEN_IDCHGIEN_Msk | HSOTG_INTEN_ROLECHGIEN_Msk);

    usbh_core_init();
    usbh_install_conn_callback(connect_func, disconnect_func);
    usbh_hid_init();
    memset(g_hid_list, 0, sizeof(g_hid_list));

    while (1)
    {
        if (HSOTG_GET_STATUS(HSOTG_STATUS_IDSTS_Msk))  /* B-device */
        {
            if (HSOTG_GET_STATUS(HSOTG_STATUS_BVLD_Msk))  /* plug-in */
            {
                printf("B-device\n");
                /* wait role change to host */
                HSOTG_ENABLE_INT(HSOTG_INTEN_HOSTIEN_Msk);

                HSUSBD_Open(&gsHSInfo, HID_ClassRequest, NULL);
                /* Endpoint configuration */
                HID_Init();
                HSOTG_ENABLE();
                HSOTG_ENABLE_PHY();
                HSUSBD_Start();
                NVIC_EnableIRQ(HSUSBD_IRQn);

                while (1)
                {
                    HID_UpdateMouseData();

                    if (otg_role_change)
                    {
                        printf("B Role change: B->A  %d\n", otg_role_change);
                        break;
                    }
                }
            }
        }
        else
        {
            printf("A-device\n");
            HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_HOSTIF_Msk);
            HSOTG_ENABLE_INT(HSOTG_INTEN_PDEVIEN_Msk);
            HSOTG_ENABLE();
            usbh_pooling_hubs();

            while (1)
            {
                if (usbh_pooling_hubs())
                {
                    hdev_list = usbh_hid_get_device_list();
                    hdev = hdev_list;

                    while (hdev != NULL)
                    {
                        if (is_a_new_hid_device(hdev))
                        {
                            init_hid_device(hdev);
                        }

                        hdev = hdev->next;
                    }

                    update_hid_device_list(hdev_list);
                }

                if (intcount > 3)
                {
                    usbh_hid_stop_int_read(hdev_list, 0);
                    delay_us(2000);
                    intcount = 0;
                    gStartHNP = 1;
                }

                if (gStartHNP)
                {
                    printf("do HNP...\n");
                    /* do HNP */
                    OTG_SetFeature(0x3);
                    delay_us(100000);
                    gStartHNP = 0;
                    /* port suspend */
                    HSOTG->CTL |= HSOTG_CTL_HNPREQEN_Msk;
                    HSOTG->CTL &= ~HSOTG_CTL_BUSREQ_Msk;
                    usbh_suspend();
                    printf("A suspend\n");

                    while (1)
                    {
                        if (otg_role_change)
                        {
                            printf("A Role change: A->B  %d\n", otg_role_change);
                            break;
                        }
                    }
                }

                if (otg_role_change)
                {
                    break;
                }
            }
        }

        if (otg_role_change == 2)   /* b -> a */
        {
            printf("/* b -> a */\r\n");
            HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_HOSTIF_Msk | HSOTG_INTSTS_PDEVIF_Msk);
            HSOTG_ENABLE_INT(HSOTG_INTEN_PDEVIEN_Msk);
            HSOTG_ENABLE();
            gStartHNP = intcount = 0;

            while (1)
            {
                if (usbh_pooling_hubs())
                {
                    hdev_list = usbh_hid_get_device_list();
                    hdev = hdev_list;

                    while (hdev != NULL)
                    {
                        if (is_a_new_hid_device(hdev))
                        {
                            init_hid_device(hdev);
                        }

                        hdev = hdev->next;
                    }

                    update_hid_device_list(hdev_list);
                }

                if (intcount > 5)
                {
                    usbh_hid_stop_int_read(hdev_list, 0);
                    gStartHNP = 1;
                    intcount = 0;
                }

                if (gStartHNP)
                {
                    printf("\n\nwaiting...\n\n");
                    HSOTG->CTL &= ~HSOTG_CTL_BUSREQ_Msk;
                    gStartHNP = 0;
                    delay_us(5000);
                }

                if (otg_role_change == 4)
                {
                    printf("Role change: A->B  %d\n", otg_role_change);
                    break;
                }
            }
        }

        if (otg_role_change == 3)   /* a -> b */
        {
            printf("/* a -> b */\r\n");
            /* wait role change to host */
            HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_HOSTIF_Msk | HSOTG_INTSTS_PDEVIF_Msk);
            HSOTG_ENABLE_INT(HSOTG_INTEN_HOSTIEN_Msk);

            HSUSBD_Open(&gsHSInfo, HID_ClassRequest, NULL);
            /* Endpoint configuration */
            HID_Init();
            HSUSBD_Start();
            NVIC_EnableIRQ(HSUSBD_IRQn);

            while (1)
            {
                HID_UpdateMouseData();

                if (otg_role_change == 4)
                {
                    printf("Role change: B->A  %d\n", otg_role_change);
                    break;
                }
            }
        }

        if (otg_role_change == 4)
            break;
    }

    printf("\n finish HNP testing ..\n");

    while (1);
}
