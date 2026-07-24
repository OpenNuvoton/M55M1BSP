/**************************************************************************//**
 * @file     ohci_1.c
 * @version  V1.10
 * @brief   USB Host library OHCI (USB 1.1) host controller driver.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "hub.h"
#include "ohci.h"

/// @cond HIDDEN_SYMBOLS
#ifdef ENABLE_OHCI1
#define _ohci_port      (1UL)
#define _ohci           _ohci1
#define ohci_driver     ohci1_driver

#define OHCI_IRQHandler   USBH1_IRQHandler
extern NVT_ITCM void OHCI_IRQHandler(void);

#define ENABLE_OHCI_IRQ   ENABLE_OHCI1_IRQ
#define DISABLE_OHCI_IRQ  DISABLE_OHCI1_IRQ

#define _hcca                       _ohci1_hcca
#define _Ied                        _ohci1_Ied
#define ed_remove_list              _ohci1_ed_remove_list
#define add_to_ED_remove_list       _ohci1_add_to_ED_remove_list
#define ohci_reset                  _ohci1_ohci_reset
#define init_hcca_int_table         _ohci1_init_hcca_int_table
#define get_int_tree_head_node      _ohci1_get_int_tree_head_node
#define get_ohci_interval           _ohci1_get_ohci_interval
#define ed_make_info                _ohci1_ed_make_info
#define write_td                    _ohci1_write_td
#define ohci_find_device_by_port    _ohci1_ohci_find_device_by_port
#define ohci_init                   _ohci1_ohci_init
#define ohci_suspend                _ohci1_ohci_suspend
#define ohci_resume                 _ohci1_ohci_resume
#define ohci_shutdown               _ohci1_ohci_shutdown
#define ohci_ctrl_xfer              _ohci1_ohci_ctrl_xfer
#define ohci_bulk_xfer              _ohci1_ohci_bulk_xfer
#define ohci_int_xfer               _ohci1_ohci_int_xfer
#define ohci_iso_xfer               _ohci1_ohci_iso_xfer
#define ohci_quit_xfer              _ohci1_ohci_quit_xfer
#define ohci_rh_port_reset          _ohci1_ohci_rh_port_reset
#define ohci_rh_polling             _ohci1_ohci_rh_polling
#define remove_ed                   _ohci1_remove_ed
#define td_done                     _ohci1_td_done
#define dump_ohci_regs              _ohci1_dump_ohci_regs
#define dump_ohci_ports             _ohci1_dump_ohci_ports


//#define TD_debug   printf
#define TD_debug(...)

//#define ED_debug   printf
#define ED_debug(...)

#define PORT_CNT    (_ohci->HcRhDescriptorA & 0xfU)

#ifdef __ICCARM__
    #if (NVT_DCACHE_ON == 1)
        /* USB Host hcca area are placed in a non-cacheable region */
        #pragma data_alignment = 256
        NVT_NONCACHEABLE static HCCA_T _hcca;
    #else
        #pragma data_alignment = 256
        static HCCA_T _hcca;
    #endif
#else
    #if (NVT_DCACHE_ON == 1)
        /* USB Host hcca area are placed in a non-cacheable region */
        NVT_NONCACHEABLE static HCCA_T _hcca __attribute__((aligned(256)));
    #else
        static HCCA_T _hcca __attribute__((aligned(256)));
    #endif
#endif

static ED_T   *_Ied[6];

static ED_T  *ed_remove_list;

static void add_to_ED_remove_list(ED_T *ed)
{
    const ED_T  *p;

    ED_debug("add_to_ED_remove_list - 0x%x (0x%x)\n", (int)ed, ed->Info);
    DISABLE_OHCI_IRQ();

    /* check if this ED found in ed_remove_list */
    p = ed_remove_list;

    while (p)
    {
        if (p == ed)
        {
            ENABLE_OHCI_IRQ();              /* This ED found in ed_remove_list            */
            return;                         /* do nothing                                 */
        }

        p = p->next;
    }

    ed->Info |= ED_SKIP;                    /* ask OHCI controller skip this ED           */
    ed->next = ed_remove_list;
    ed_remove_list = ed;                    /* insert to the head of ed_remove_list       */
    ENABLE_OHCI_IRQ();
    _ohci->HcInterruptStatus = USBH_HcInterruptStatus_SF_Msk;
    _ohci->HcInterruptEnable |= USBH_HcInterruptEnable_SF_Msk;
    delay_us(2000);                         /* Full speed wait 2 ms is enough             */
}

static int ohci_reset(void)
{

    /* Disable HC interrupts  */
    _ohci->HcInterruptDisable = USBH_HcInterruptDisable_MIE_Msk;

    /* HC Reset requires max 10 ms delay  */
    _ohci->HcControl = 0;
    _ohci->HcCommandStatus = USBH_HcCommandStatus_HCR_Msk;

    delay_us(10000);

    /* Check if OHCI reset completed? */
    if ((_ohci->HcCommandStatus & USBH_HcCommandStatus_HCR_Msk) != 0U)
    {
        USB_error("Error! - USB OHCI reset timed out!\n");
        return -1;
    }

    _ohci->HcRhStatus = USBH_HcRhStatus_OCI_Msk | USBH_HcRhStatus_LPS_Msk;

    _ohci->HcControl = HCFS_RESET;

    delay_us(10000);

    /* Check if OHCI reset completed? */
    if ((_ohci->HcCommandStatus & USBH_HcCommandStatus_HCR_Msk) != 0U)
    {
        USB_error("Error! - USB HC reset timed out!\n");
        return -1;
    }

    return 0;
}

static void init_hcca_int_table(void)
{
    int    i;
    int    idx;

    (void)memset(_hcca.int_table, 0, sizeof(_hcca.int_table));

    for (i = 5; i >= 0; i--)                     /* interval = i^2                        */
    {
        _Ied[i] = alloc_ohci_ED();
        _Ied[i]->Info = ED_SKIP;

        int    interval;
        unsigned int tmp = (unsigned int)1U << (unsigned int)i;
        interval = (int)tmp;

        for (idx = interval - 1; idx < 32; idx += interval)
        {
            if (_hcca.int_table[idx] == 0U)       /* is empty list, insert directly        */
            {
                _hcca.int_table[idx] = (uint32_t)_Ied[i];
            }
            else
            {
                ED_T   *ed_p;
                ed_p = (ED_T *)_hcca.int_table[idx];

                while ((ed_p != _Ied[i]) && (ed_p->NextED != 0U))
                {
                    ed_p = (ED_T *)ed_p->NextED;
                }

                if ((ed_p != _Ied[i]) && (ed_p->NextED == 0U))
                {
                    ed_p->NextED = (uint32_t)_Ied[i];  /* reach end of list, chain it    */
                }
            }
        }
    }
}

static ED_T *get_int_tree_head_node(const int interval)
{
    int    i;
    int    interval_tmp = interval;

    for (i = 0; i < 5; i++)
    {
        interval_tmp >>= 1;

        if (interval_tmp == 0)
        {
            return _Ied[i];
        }
    }

    return _Ied[5];                         /* for interval >= 32                         */
}

static int get_ohci_interval(const int interval)
{
    int    i;
    int    bInterval = 1;
    int    interval_tmp = interval;

    for (i = 0; i < 5; i++)
    {
        interval_tmp >>= 1;

        if (interval_tmp == 0)
        {
            return bInterval;
        }

        bInterval *= 2;
    }

    return 32;                              /* for interval >= 32                         */
}

static int  ohci_init(void)
{
    uint32_t    fminterval;

    if (ohci_reset() < 0)
    {
        return -1;
    }

    ed_remove_list = USBNULL;

    init_hcca_int_table();

    /* Tell the controller where the control and bulk lists are
     * The lists are empty now. */
    _ohci->HcControlHeadED = 0;             /* control ED list head                       */
    _ohci->HcBulkHeadED = 0;                /* bulk ED list head                          */

    _ohci->HcHCCA = (uint32_t)&_hcca;       /* HCCA area                                  */

    /* periodic start 90% of frame interval       */
    fminterval = 0x2edf;                    /* 11,999                                     */
    _ohci->HcPeriodicStart = (fminterval * 9U) / 10U;

    /* set FSLargestDataPacket, 10,104 for 0x2edf frame interval */
    fminterval |= ((uint32_t)(((fminterval - 210U) * 6U) / 7U) << 16U);
    _ohci->HcFmInterval = fminterval;

    _ohci->HcLSThreshold = 0x628;

    /* start controller operations                */
    _ohci->HcControl = HCFS_OPER | (0x3U << USBH_HcControl_CBSR_Pos);

    if (_ohci->HcRhDescriptorA & USBH_HcRhDescriptorA_PSM_Msk)
    {
        uint32_t    portcnt = PORT_CNT;

        /* is per-port powered */
        if (portcnt == 1U)
        {
            _ohci->HcRhDescriptorB = 0x20000U;
        }
        else
        {
            _ohci->HcRhDescriptorB = 0x7E0000U;
        }

        uint32_t    i;

        for (i = 0; i < portcnt; i++)
        {
            _ohci->HcRhPortStatus[i] = USBH_HcRhPortStatus_PPS_Msk;
        }
    }
    else
    {
        /* is global port powered */
        _ohci->HcRhDescriptorA = (_ohci->HcRhDescriptorA | ((uint32_t)1U << 9U)) & ~USBH_HcRhDescriptorA_PSM_Msk;
        _ohci->HcRhStatus = USBH_HcRhStatus_LPSC_Msk;
    }

    _ohci->HcInterruptEnable = USBH_HcInterruptEnable_MIE_Msk | USBH_HcInterruptEnable_WDH_Msk | USBH_HcInterruptEnable_SF_Msk;

    /* POTPGT delay is bits 24-31, in 20 ms units.                                        */
    delay_us(20000);

    return 0;
}

static void ohci_suspend(void)
{
    /* set port suspend if connected */
    if (_ohci->HcRhPortStatus[0] & 0x1U)
    {
        _ohci->HcRhPortStatus[0] = 0x4;
    }

    /* enable Device Remote Wakeup */
    _ohci->HcRhStatus |= USBH_HcRhStatus_DRWE_Msk;

    /* enable USBH RHSC interrupt for system wakeup */
    _ohci->HcInterruptEnable |=  USBH_HcInterruptEnable_RHSC_Msk | USBH_HcInterruptEnable_RD_Msk;

    /* set Host Controller enter suspend state */
    _ohci->HcControl = (_ohci->HcControl & ~USBH_HcControl_HCFS_Msk) | (3U << USBH_HcControl_HCFS_Pos);
}

static void ohci_resume(void)
{
    _ohci->HcControl = (_ohci->HcControl & ~USBH_HcControl_HCFS_Msk) | (1U << USBH_HcControl_HCFS_Pos);
    _ohci->HcControl = (_ohci->HcControl & ~USBH_HcControl_HCFS_Msk) | (2U << USBH_HcControl_HCFS_Pos);

    if (_ohci->HcRhPortStatus[0] & 0x4U)
    {
        _ohci->HcRhPortStatus[0] = 0x8;
    }
}

static void ohci_shutdown(void)
{
    ohci_suspend();
    DISABLE_OHCI_IRQ();
#ifndef OHCI_PER_PORT_POWER
    _ohci->HcRhStatus = USBH_HcRhStatus_LPS_Msk;
#endif
}

/*
 *  Quit current trasnfer via UTR or hardware EP.
 */
static int ohci_quit_xfer(UTR_T *utr, EP_INFO_T *ep)
{
    ED_T       *ed;

    if (utr != USBNULL)
    {
        if (utr->ep == USBNULL)
        {
            return USBH_ERR_NOT_FOUND;
        }

        ed = (ED_T *)(utr->ep->hw_pipe);

        if (!ed)
        {
            return USBH_ERR_NOT_FOUND;
        }

        /* add the endpoint to remove list, it will be removed on the next start of frame */
        add_to_ED_remove_list(ed);
        utr->ep->hw_pipe = USBNULL;
    }

    if ((ep != USBNULL) && (ep->hw_pipe != USBNULL))
    {
        ed = (ED_T *)(ep->hw_pipe);
        /* add the endpoint to remove list, it will be removed on the next start of frame */
        add_to_ED_remove_list(ed);
        ep->hw_pipe = USBNULL;
    }

    return 0;
}

static uint32_t ed_make_info(UDEV_T *udev, const EP_INFO_T *ep)
{
    uint32_t  info;

    if (ep == USBNULL)                             /* is a control endpoint                  */
    {
        /* control endpoint direction is from TD  */
        if (udev->descriptor.bMaxPacketSize0 == 0)   /* is 0 if device descriptor still not obtained. */
        {
            if (udev->speed == SPEED_LOW)
            {
                /* give a default maximum packet size     */
                udev->descriptor.bMaxPacketSize0 = 8;
            }
            else
            {
                udev->descriptor.bMaxPacketSize0 = 64;
            }
        }

        info = ((uint32_t)udev->descriptor.bMaxPacketSize0 << 16U) /* Control endpoint Maximum Packet Size from device descriptor */
               | ED_DIR_BY_TD                   /* Direction (Get direction From TD)      */
               | ED_FORMAT_GENERAL              /* General format                         */
               | (0U << ED_CTRL_EN_Pos);        /* Endpoint address 0                     */
    }
    else                                        /* Other endpoint direction is from endpoint descriptor */
    {
        info = ((uint32_t)ep->wMaxPacketSize << 16U);      /* Maximum Packet Size from endpoint      */

        info |= ((uint32_t)(ep->bEndpointAddress & 0xfU) << ED_CTRL_EN_Pos);   /* Endpoint Number    */

        if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN)
        {
            info |= ED_DIR_IN;
        }
        else
        {
            info |= ED_DIR_OUT;
        }

        if ((ep->bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_ISO)
        {
            info |= ED_FORMAT_ISO;
        }
        else
        {
            info |= ED_FORMAT_GENERAL;
        }
    }

    info |= ((udev->speed == SPEED_LOW)  ? ED_SPEED_LOW : ED_SPEED_FULL);  /* Speed       */
    info |= (udev->dev_num);                  /* Function Address                         */

    return info;
}

static void write_td(TD_T *td, uint32_t info, uint8_t *buff, uint32_t data_len)
{
    td->Info = info;
    td->CBP  = (uint32_t)((!buff || !data_len) ? 0 : buff);
    td->BE   = (uint32_t)((!buff || !data_len) ? 0U : ((uint32_t)buff + data_len - 1U));
    td->buff_start = td->CBP;
    // TD_debug("TD [0x%x]: 0x%x, 0x%x, 0x%x\n", (int)td, td->Info, td->CBP, td->BE);
}

static int ohci_ctrl_xfer(UTR_T *utr)
{
    UDEV_T  *udev;
    ED_T    *ed;
    TD_T    *td_setup;
    TD_T    *td_data;
    TD_T    *td_status;
    uint32_t  info;

    udev = utr->udev;

    /*------------------------------------------------------------------------------------*/
    /*  Allocate ED and TDs                                                               */
    /*------------------------------------------------------------------------------------*/
    td_setup = alloc_ohci_TD(utr);

    if (utr->data_len > 0U)
    {
        td_data = alloc_ohci_TD(utr);
    }
    else
    {
        td_data = USBNULL;
    }

    td_status = alloc_ohci_TD(utr);

    if (td_status == USBNULL)
    {
        free_ohci_TD(td_setup);

        if (utr->data_len > 0U)
        {
            free_ohci_TD(td_data);
        }

        return USBH_ERR_MEMORY_OUT;
    }

    /* Check if there's any transfer pending on this endpoint... */
    if (udev->ep0.hw_pipe == USBNULL)
    {
        ed = alloc_ohci_ED();

        if (ed == USBNULL)
        {
            free_ohci_TD(td_setup);
            free_ohci_TD(td_status);

            if (utr->data_len > 0U)
            {
                free_ohci_TD(td_data);
            }

            return USBH_ERR_MEMORY_OUT;
        }
    }
    else
    {
        ed = (ED_T *)udev->ep0.hw_pipe;
    }

    /*------------------------------------------------------------------------------------*/
    /* prepare SETUP stage TD                                                             */
    /*------------------------------------------------------------------------------------*/
    info = TD_CC | TD_T_DATA0 | TD_TYPE_CTRL;
    write_td(td_setup, info, (uint8_t *)&utr->setup, 8);
    td_setup->ed = ed;

    /*------------------------------------------------------------------------------------*/
    /* prepare DATA stage TD                                                              */
    /*------------------------------------------------------------------------------------*/
    if (utr->data_len > 0U)
    {
        if ((utr->setup.bmRequestType & 0x80U) == REQ_TYPE_OUT)
        {
            info = (TD_CC | TD_R | TD_DP_OUT | TD_T_DATA1 | TD_TYPE_CTRL | TD_CTRL_DATA);
        }
        else
        {
            info = (TD_CC | TD_R | TD_DP_IN | TD_T_DATA1 | TD_TYPE_CTRL | TD_CTRL_DATA);
        }

        write_td(td_data, info, utr->buff, utr->data_len);
        td_data->ed = ed;
        td_setup->NextTD = (uint32_t)td_data;
        td_setup->next   = td_data;
        td_data->NextTD  = (uint32_t)td_status;
        td_data->next    = td_status;
    }
    else
    {
        td_setup->NextTD = (uint32_t)td_status;
        td_setup->next = td_status;
    }

    /*------------------------------------------------------------------------------------*/
    /* prepare STATUS stage TD                                                            */
    /*------------------------------------------------------------------------------------*/
    ed->Info = ed_make_info(udev, USBNULL);

    if ((utr->setup.bmRequestType & 0x80U) == REQ_TYPE_OUT)
    {
        info = (TD_CC | TD_DP_IN | TD_T_DATA1 | TD_TYPE_CTRL);
    }
    else
    {
        info = (TD_CC | TD_DP_OUT | TD_T_DATA1 | TD_TYPE_CTRL);
    }

    write_td(td_status, info, USBNULL, 0);
    td_status->ed = ed;
    td_status->NextTD = 0;
    td_status->next = USBNULL;

    /*------------------------------------------------------------------------------------*/
    /* prepare ED                                                                         */
    /*------------------------------------------------------------------------------------*/
    ed->TailP = 0;
    ed->HeadP = (uint32_t)td_setup;
    ed->Info = ed_make_info(udev, USBNULL);
    ed->NextED = 0;

    //TD_debug("TD SETUP [0x%x]: 0x%x, 0x%x, 0x%x, 0x%x\n", (int)td_setup, td_setup->Info, td_setup->CBP, td_setup->BE, td_setup->NextTD);
    //if (td_data)
    //    TD_debug("TD DATA  [0x%x]: 0x%x, 0x%x, 0x%x, 0x%x\n", (int)td_data, td_data->Info, td_data->CBP, td_data->BE, td_data->NextTD);
    //TD_debug("TD STATUS [0x%x]: 0x%x, 0x%x, 0x%x, 0x%x\n", (int)td_status, td_status->Info, td_status->CBP, td_status->BE, td_status->NextTD);
    ED_debug("Xfer ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);

    if (utr->data_len > 0U)
    {
        utr->td_cnt = 3;
    }
    else
    {
        utr->td_cnt = 2;
    }

    utr->ep = &udev->ep0;                   /* driver can find EP from UTR                */
    udev->ep0.hw_pipe = (void *)ed;         /* driver can find ED from EP                 */

    /*------------------------------------------------------------------------------------*/
    /*  Start transfer                                                                    */
    /*------------------------------------------------------------------------------------*/
    DISABLE_OHCI_IRQ();
    _ohci->HcControlHeadED = (uint32_t)ed;                   /* Link ED to OHCI           */
    _ohci->HcControl |= USBH_HcControl_CLE_Msk;              /* enable control list       */
    ENABLE_OHCI_IRQ();
    _ohci->HcCommandStatus = USBH_HcCommandStatus_CLF_Msk;   /* start Control list        */

    return 0;
}

static int ohci_bulk_xfer(UTR_T *utr)
{
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    ED_T       *ed;
    TD_T       *td;
    TD_T       *td_list = USBNULL;
    uint32_t   info;
    uint32_t   data_len;
    uint32_t   xfer_len;
    uint32_t   buff_pos;
    int8_t     bIsNewED = 0;

    /*------------------------------------------------------------------------------------*/
    /*  Check if there's uncompleted transfer on this endpoint...                         */
    /*  Prepare ED                                                                        */
    /*------------------------------------------------------------------------------------*/
    info = ed_make_info(udev, ep);

    /* Check if there's any transfer pending on this endpoint... */
    ed = (ED_T *)_ohci->HcBulkHeadED;       /* get the head of bulk endpoint list         */

    while (ed != USBNULL)
    {
        if (ed->Info == info)               /* have transfer of this EP not completed?    */
        {
            if ((ed->HeadP & 0xFFFFFFF0U) != (ed->TailP & 0xFFFFFFF0U))
            {
                return USBH_ERR_OHCI_EP_BUSY;     /* endpoint is busy                     */
            }
            else
            {
                break;                      /* ED already there...                        */
            }
        }

        ed = (ED_T *)ed->NextED;
    }

    if (ed == USBNULL)
    {
        bIsNewED = 1;
        ed = alloc_ohci_ED();               /* allocate an Endpoint Descriptor            */

        if (ed == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }

        ed->Info = info;
        ed->HeadP = 0;
        ED_debug("Link BULK ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);
    }

    ep->hw_pipe = (void *)ed;

    /*------------------------------------------------------------------------------------*/
    /*  Prepare TDs                                                                       */
    /*------------------------------------------------------------------------------------*/
    utr->td_cnt = 0;
    data_len = utr->data_len;
    buff_pos = 0U;

    do
    {
        if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT)
        {
            info = (TD_CC | TD_R | TD_DP_OUT | TD_TYPE_BULK);
        }
        else
        {
            info = (TD_CC | TD_R | TD_DP_IN | TD_TYPE_BULK);
        }

        info &= ~((uint32_t)1U << 25U);                 /* Data toggle from ED toggleCarry bit        */

        if (data_len > 4096U)
        {
            /* maximum transfer length is 4K for each TD  */
            xfer_len = 4096U;
        }
        else
        {
            xfer_len = data_len;            /* remaining data length < 4K                 */
        }

        td = alloc_ohci_TD(utr);            /* allocate a TD                              */

        if (td == USBNULL)
        {
            /* mem_out: free all allocated TDs and ED */
            while (td_list != USBNULL)
            {
                td = td_list;
                td_list = (TD_T *)td_list->NextTD;
                free_ohci_TD(td);
            }

            free_ohci_ED(ed);
            return USBH_ERR_MEMORY_OUT;
        }

        /* fill this TD                               */
        uint8_t  *td_buff;

        td_buff = USBNULL;

        if ((utr->buff != USBNULL) && (xfer_len > 0U))
        {
            td_buff = &utr->buff[buff_pos];
        }

        write_td(td, info, td_buff, xfer_len);
        td->ed = ed;

        utr->td_cnt++;                      /* increase TD count, for recalim counter     */

        buff_pos += xfer_len;
        data_len -= xfer_len;

        /* chain to end of TD list */
        if (td_list == USBNULL)
        {
            td_list = td;
        }
        else
        {
            TD_T       *td_p;
            td_p = td_list;

            while (td_p->NextTD != 0U)
            {
                td_p = (TD_T *)td_p->NextTD;
            }

            td_p->NextTD = (uint32_t)td;
        }
    } while (data_len > 0U);

    /*------------------------------------------------------------------------------------*/
    /*  Start transfer                                                                    */
    /*------------------------------------------------------------------------------------*/
    utr->status = 0;
    DISABLE_OHCI_IRQ();
    ed->HeadP = (ed->HeadP & 0x2U) | (uint32_t)td_list;       /* keep toggleCarry bit      */

    if (bIsNewED)
    {
        ed->HeadP = (uint32_t)td_list;
        /* Link ED to OHCI Bulk List */
        ed->NextED = _ohci->HcBulkHeadED;
        _ohci->HcBulkHeadED = (uint32_t)ed;
    }

    delay_us(3000);

    ENABLE_OHCI_IRQ();
    _ohci->HcControl |= USBH_HcControl_BLE_Msk;              /* enable bulk list          */
    _ohci->HcCommandStatus = USBH_HcCommandStatus_BLF_Msk;   /* start bulk list           */

    return 0;
}

static int ohci_int_xfer(UTR_T *utr)
{
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    ED_T       *ed;
    ED_T       *ied;
    TD_T       *td;
    const TD_T       *td_new;
    uint32_t   info;
    int8_t     bIsNewED = 0;

    if (utr->data_len > 64U)
    {
        /* USB 1.1 interrupt transfer maximum packet size is 64 */
        return USBH_ERR_INVALID_PARAM;
    }

    td_new = alloc_ohci_TD(USBNULL);        /* allocate a TD for dummy TD                     */

    if (td_new == USBNULL)
    {
        return USBH_ERR_MEMORY_OUT;
    }

    ied = get_int_tree_head_node(ep->bInterval);  /* get head node of this interval       */

    /*------------------------------------------------------------------------------------*/
    /*  Find if this ED was already in the list                                           */
    /*------------------------------------------------------------------------------------*/
    info = ed_make_info(udev, ep);
    ed = ied;

    while (ed != USBNULL)
    {
        if (ed->Info == info)
        {
            break;                          /* Endpoint found                             */
        }

        ed = (ED_T *)ed->NextED;
    }

    if (ed == USBNULL)                         /* ED not found, create it                    */
    {
        bIsNewED = 1;
        ed = alloc_ohci_ED();               /* allocate an Endpoint Descriptor            */

        if (ed == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }

        ed->Info = info;
        ed->HeadP = 0;
        ed->bInterval = ep->bInterval;

        td = alloc_ohci_TD(USBNULL);           /* allocate the initial  dummy TD for ED      */

        if (td == USBNULL)
        {
            free_ohci_ED(ed);
            free_ohci_TD(td_new);
            return USBH_ERR_MEMORY_OUT;
        }

        ed->HeadP = (uint32_t)td;           /* Let both HeadP and TailP point to dummy TD */
        ed->TailP = ed->HeadP;
    }
    else
    {
        td = (TD_T *)(ed->TailP & ~0xfU);    /* TailP always point to the dummy TD         */
    }

    ep->hw_pipe = (void *)ed;

    /*------------------------------------------------------------------------------------*/
    /*  Prepare TD                                                                        */
    /*------------------------------------------------------------------------------------*/
    if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT)
    {
        info = (TD_CC | TD_R | TD_DP_OUT | TD_TYPE_INT);
    }
    else
    {
        info = (TD_CC | TD_R | TD_DP_IN | TD_TYPE_INT);
    }

    /* Keep data toggle                               */
    info = (info & ~((uint32_t)1U << 25U)) | (td->Info & ((uint32_t)1U << 25U));

    /* fill this TD                                   */
    write_td(td, info, utr->buff, utr->data_len);
    td->ed = ed;
    td->NextTD = (uint32_t)td_new;
    td->utr = utr;
    utr->td_cnt = 1;                    /* increase TD count, for recalim counter     */
    utr->status = 0;

    /*------------------------------------------------------------------------------------*/
    /*  Hook ED and TD list to HCCA interrupt table                                       */
    /*------------------------------------------------------------------------------------*/
    DISABLE_OHCI_IRQ();

    ed->TailP = (uint32_t)td_new;

    if (bIsNewED)
    {
        /* Add to list of the same interval */
        ed->NextED = ied->NextED;
        ied->NextED = (uint32_t)ed;
    }

    ENABLE_OHCI_IRQ();

    //printf("Link INT ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);

    _ohci->HcControl |= USBH_HcControl_PLE_Msk;              /* periodic list enable      */
    return 0;
}

static int ohci_iso_xfer(UTR_T *utr)
{
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    ED_T       *ed;
    ED_T       *ied;
    TD_T       *td;
    TD_T       *td_list;
    TD_T       *last_td;
    uint32_t   i;
    uint32_t   info;
    int8_t     bIsNewED = 0;

    ied = get_int_tree_head_node(ep->bInterval);  /* get head node of this interval       */

    /*------------------------------------------------------------------------------------*/
    /*  Find if this ED was already in the list                                           */
    /*------------------------------------------------------------------------------------*/
    info = ed_make_info(udev, ep);
    ed = ied;

    while (ed != USBNULL)
    {
        if (ed->Info == info)
        {
            break;                          /* Endpoint found                             */
        }

        ed = (ED_T *)ed->NextED;
    }

    if (ed == USBNULL)                         /* ED not found, create it                    */
    {
        bIsNewED = 1;
        ed = alloc_ohci_ED();               /* allocate an Endpoint Descriptor            */

        if (ed == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }

        ed->Info = info;
        ed->HeadP = 0;
        ed->bInterval = ep->bInterval;
    }
    else
    {
        ep->hw_pipe = (void *)ed;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Prepare TDs                                                                       */
    /*------------------------------------------------------------------------------------*/
    if (utr->bIsoNewSched)
    {
        /* Is the starting of isochronous streaming?  */
        ed->next_sf = _hcca.frame_no + (uint16_t)OHCI_ISO_DELAY;
    }

    utr->td_cnt = 0;
    utr->iso_sf = ed->next_sf;

    last_td = USBNULL;
    td_list = USBNULL;

    for (i = 0; i < IF_PER_UTR; i++)
    {
        utr->iso_status[i] = USBH_ERR_NOT_ACCESS1;

        td = alloc_ohci_TD(utr);            /* allocate a TD                              */

        if (td == USBNULL)
        {
            /* mem_out: free all allocated TDs and ED */
            while (td_list != USBNULL)
            {
                td = td_list;
                td_list = (TD_T *)td_list->NextTD;
                free_ohci_TD(td);
            }

            free_ohci_ED(ed);
            return USBH_ERR_MEMORY_OUT;
        }

        /* fill this TD                               */
        uint32_t   buff_addr;
        buff_addr = (uint32_t)(utr->iso_buff[i]);
        td->Info = (TD_CC | TD_TYPE_ISO) | ed->next_sf;
        ed->next_sf += (uint16_t)get_ohci_interval(ed->bInterval);
        td->CBP  = buff_addr & ~0xFFFU;
        td->BE   = buff_addr + (uint32_t)utr->iso_xlen[i] - 1U;
        td->PSW[0] = 0xE000U | (buff_addr & 0xFFFU);

        td->ed = ed;
        utr->td_cnt++;                      /* increase TD count, for reclaim counter     */

        /* chain to end of TD list */
        if (td_list == USBNULL)
        {
            td_list = td;
        }
        else
        {
            last_td->NextTD = (uint32_t)td;
        }

        last_td = td;
    };

    /*------------------------------------------------------------------------------------*/
    /*  Hook ED and TD list to HCCA interrupt table                                       */
    /*------------------------------------------------------------------------------------*/
    utr->status = 0;

    DISABLE_OHCI_IRQ();

    if ((ed->HeadP & ~0x3U) == 0U)
    {
        ed->HeadP = (ed->HeadP & 0x2U) | (uint32_t)td_list;   /* keep toggleCarry bit      */
    }
    else
    {
        /* find the tail of TDs under this ED */
        td = (TD_T *)(ed->HeadP & ~0x3U);

        while (td->NextTD != 0U)
        {
            td = (TD_T *)td->NextTD;
        }

        td->NextTD = (uint32_t)td_list;
    }

    if (bIsNewED)
    {
        /* Add to list of the same interval */
        ed->NextED = ied->NextED;
        ied->NextED = (uint32_t)ed;
    }

    ENABLE_OHCI_IRQ();
    ED_debug("Link ISO ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);
    _ohci->HcControl |= USBH_HcControl_PLE_Msk | USBH_HcControl_IE_Msk;  /* enable periodic list and isochronous transfer */

    return 0;
}

static UDEV_T *ohci_find_device_by_port(int port)
{
    UDEV_T  *udev;

    udev = g_udev_list;

    while (udev != USBNULL)
    {
        if ((udev->parent == USBNULL) && (udev->port_num == (uint8_t)port) &&
                ((udev->speed == SPEED_LOW) || (udev->speed == SPEED_FULL)))
        {
            return udev;
        }

        udev = udev->next;
    }

    return USBNULL;
}

static int ohci_rh_port_reset(int port)
{
    int       retry;
    int       reset_time;
    int8_t    bResetDone = 0;

    reset_time = PORT_RESET_TIME_MS;

    for (retry = 0; (retry < PORT_RESET_RETRY) && (bResetDone == 0); retry++)
    {
        _ohci->HcRhPortStatus[port] = USBH_HcRhPortStatus_PRS_Msk;

        uint32_t  t0;
        t0 = get_ticks();

        while ((get_ticks() - t0) < ((uint32_t)((uint32_t)reset_time / 10U) + 1U))
        {
            /*
             *  If device is disconnected or port enabled, we can stop port reset.
             */
            if (((_ohci->HcRhPortStatus[port] & USBH_HcRhPortStatus_CCS_Msk) == 0U) ||
                    ((_ohci->HcRhPortStatus[port] & (USBH_HcRhPortStatus_PES_Msk | USBH_HcRhPortStatus_CCS_Msk)) == (USBH_HcRhPortStatus_PES_Msk | USBH_HcRhPortStatus_CCS_Msk)))
            {
                bResetDone = 1;
                break;
            }
        }

        reset_time += PORT_RESET_RETRY_INC_MS;
    }

    if (bResetDone == 0)
    {
        USB_debug("OHCI port %d - port reset failed!\n", port + 1);
        return USBH_ERR_PORT_RESET;
    }

    if ((_ohci->HcRhPortStatus[port] & USBH_HcRhPortStatus_CCS_Msk) == 0U)  /* check again if device disconnected */
    {
        _ohci->HcRhPortStatus[port] = USBH_HcRhPortStatus_CSC_Msk;         /* clear CSC */
        return USBH_ERR_DISCONNECTED;
    }

    return USBH_OK;                                                        /* port reset success */
}

static int ohci_rh_polling(void)
{
    int       i;
    int       change = 0;
    UDEV_T    *udev;
    int       ret;

    for (i = 0; i < 1; i++)
    {

#if (_ohci_port == 1UL) && defined (ENABLE_EHCI)

        //2024/3/22
        if ((_ehci->UPSCR[0] & HSUSBH_UPSCR_SUSPEND_Msk) == (HSUSBH_UPSCR_SUSPEND_Msk))
        {
            continue;
        }

#endif

        /* clear unwanted port change status */
        _ohci->HcRhPortStatus[i] = USBH_HcRhPortStatus_OCIC_Msk | USBH_HcRhPortStatus_PRSC_Msk |
                                   USBH_HcRhPortStatus_PSSC_Msk | USBH_HcRhPortStatus_PESC_Msk;

        if ((_ohci->HcRhPortStatus[i] & USBH_HcRhPortStatus_CSC_Msk) == 0U)
        {
            continue;
        }

        /*--------------------------------------------------------------------------------*/
        /*  connect status change                                                         */
        /*--------------------------------------------------------------------------------*/

        _ohci->HcRhPortStatus[i] = USBH_HcRhPortStatus_CSC_Msk;     /* clear CSC          */

        if (_ohci->HcRhPortStatus[i] & USBH_HcRhPortStatus_CCS_Msk)
        {
            /*----------------------------------------------------------------------------*/
            /*  First of all, check if there's any previously connected device.           */
            /*----------------------------------------------------------------------------*/
            while (1)
            {
                udev = ohci_find_device_by_port(_ohci_port);

                if (udev == USBNULL)
                {
                    break;
                }

                disconnect_device(udev);
            }

            if (ohci_rh_port_reset(i) != USBH_OK)
            {
                continue;
            }

            /*
             *  Port reset success...
             */
            udev = alloc_device();

            if (udev == USBNULL)
            {
                continue;
            }

            udev->parent = USBNULL;
            udev->port_num = _ohci_port;

            if (_ohci->HcRhPortStatus[i] & USBH_HcRhPortStatus_LSDA_Msk)
            {
                udev->speed = SPEED_LOW;
            }
            else
            {
                udev->speed = SPEED_FULL;
            }

            udev->hc_driver = &ohci_driver;

            ret = connect_device(udev);

            if (ret < 0)
            {
                USB_error("connect_device error! [%d]\n", ret);
                free_device(udev);
            }

            change = 1;
        }
        else
        {
            /*
             *  Device disconnected
             */
            while (1)
            {
                udev = ohci_find_device_by_port(_ohci_port);

                if (udev == USBNULL)
                {
                    break;
                }

                disconnect_device(udev);
            }

            change = 1;
        }
    }

    return change;
}

static void td_done(TD_T *td)
{
    UTR_T       *utr = td->utr;
    uint32_t    info;
    int         cc;

    info = td->Info;

    TD_debug("td_done: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", (int)td, td->Info, td->CBP, td->NextTD, td->BE);

    /* ISO ... drivers see per-TD length/status */
    if ((info & TD_TYPE_Msk) == TD_TYPE_ISO)
    {
        uint16_t     sf;
        unsigned int idx;

        sf = (uint16_t)(info & 0xFFFFU);
        idx = (((uint32_t)sf + 0x10000U - (uint32_t)utr->iso_sf) & 0xFFFFU) / ((uint32_t)get_ohci_interval(td->ed->bInterval));

        if (idx >= IF_PER_UTR)
        {
            USB_error("ISO invalid index!! %d, %d\n", sf, utr->iso_sf);
        }
        else
        {
            cc = (td->PSW[0] >> 12) & 0xFU;

            if (cc == 0xF)                      /* this frame was not transferred */
            {
                USB_debug("ISO F %d N/A!\n", sf);
                utr->iso_status[idx] = USBH_ERR_SCH_OVERRUN;
            }
            else if ((cc != 0) && (cc != (int)CC_DATA_UNDERRUN))
            {
                utr->iso_status[idx] = USBH_ERR_CC_NO_ERR - cc;
            }
            else
            {
                utr->iso_status[idx] = 0;
                utr->iso_xlen[idx] = td->PSW[0] & 0x7FFU;
            }
        }
    }
    else
    {
        cc = TD_CC_GET(info);

        /* short packet is fine */
        if ((cc != (int)CC_NOERROR) && (cc != (int)CC_DATA_UNDERRUN))
        {
            USB_error("TD error, CC = 0x%x\n", cc);

            if (cc == (int)CC_STALL)
            {
                utr->status = USBH_ERR_STALL;
            }
            else
            {
                utr->status = USBH_ERR_TRANSFER;
            }
        }

        switch (info & TD_TYPE_Msk)
        {
            case TD_TYPE_CTRL:
                if (info & TD_CTRL_DATA)
                {
                    if (td->CBP == 0U)
                    {
                        utr->xfer_len += td->BE - td->buff_start + 1U;
                    }
                    else
                    {
                        utr->xfer_len += td->CBP - td->buff_start;
                    }
                }

                break;

            case TD_TYPE_BULK:
            case TD_TYPE_INT:
                if (td->CBP == 0U)
                {
                    utr->xfer_len += td->BE - td->buff_start + 1U;
                }
                else
                {
                    utr->xfer_len += td->CBP - td->buff_start;
                }

                break;

            default:
                break;
        }
    }

    utr->td_cnt--;

    /* If all TDs are done, call-back to requester. */
    if (utr->td_cnt == 0)
    {
        utr->bIsTransferDone = 1;

        if (utr->func)
        {
            utr->func(utr);
        }
    }
}

/* in IRQ context */
static void remove_ed(void)
{
    ED_T      *ed;
    ED_T      *ed_p;
    ED_T      *ied;
    TD_T      *td;
    TD_T      *td_next;

    while (ed_remove_list != USBNULL)
    {
        ED_debug("Remove ED: 0x%x, %d\n", (int)ed_remove_list, ed_remove_list->bInterval);
        ed_p = ed_remove_list;

        int       found = 0;

        /*--------------------------------------------------------------------------------*/
        /*  Remove endpoint from Control List if found                                    */
        /*--------------------------------------------------------------------------------*/
        if ((ed_p->Info & ED_EP_ADDR_Msk) == 0U)
        {
            if (_ohci->HcControlHeadED == (uint32_t)ed_p)
            {
                _ohci->HcControlHeadED = (uint32_t)ed_p->NextED;
                found = 1;
            }
            else
            {
                ed = (ED_T *)_ohci->HcControlHeadED;

                while (ed != USBNULL)
                {
                    if (ed->NextED == (uint32_t)ed_p)
                    {
                        ed->NextED = ed_p->NextED;
                        found = 1;
                    }

                    ed = (ED_T *)ed->NextED;
                }
            }
        }

        /*--------------------------------------------------------------------------------*/
        /*  Remove INT or ISO endpoint from HCCA interrupt table                          */
        /*--------------------------------------------------------------------------------*/
        else if (ed_p->bInterval > (uint8_t)0)
        {
            ied = get_int_tree_head_node(ed_p->bInterval);

            ed = ied;

            while (ed != USBNULL)
            {
                if (ed->NextED == (uint32_t)ed_p)
                {
                    ed->NextED = ed_p->NextED;
                    found = 1;
                    break;
                }

                ed = (ED_T *)ed->NextED;
            }
        }

        /*--------------------------------------------------------------------------------*/
        /*  Remove endpoint from Bulk List if found                                       */
        /*--------------------------------------------------------------------------------*/
        else
        {
            if (_ohci->HcBulkHeadED == (uint32_t)ed_p)
            {
                _ohci->HcBulkHeadED = ed_p->NextED;
                found = 1;
            }
            else
            {
                ed = (ED_T *)_ohci->HcBulkHeadED;

                while (ed != USBNULL)
                {
                    if (ed->NextED == (uint32_t)ed_p)
                    {
                        ed->NextED = ed_p->NextED;
                        found = 1;
                    }

                    ed = (ED_T *)ed->NextED;
                }
            }
        }

        /*--------------------------------------------------------------------------------*/
        /*  Remove and free all TDs under this endpoint                                   */
        /*--------------------------------------------------------------------------------*/
        if (found)
        {
            td = (TD_T *)(ed_p->HeadP & ~0x3U);

            if (td != USBNULL)
            {
                while (td != USBNULL)
                {
                    UTR_T     *utr;
                    utr = td->utr;
                    td_next = (TD_T *)td->NextTD;

                    free_ohci_TD(td);

                    if (utr && utr->udev)
                    {
                        if (utr->td_cnt > 0)
                        {
                            utr->td_cnt--;

                            if (utr->td_cnt == 0)
                            {
                                utr->status = USBH_ERR_ABORT;
                                utr->bIsTransferDone = 1;

                                if (utr->func)
                                {
                                    utr->func(utr);
                                }
                            }
                        }
                    }

                    td = td_next;
                }
            }
        }

        /*
         *  Done. Remove this ED from [ed_remove_list] and free it.
         */
        ed_remove_list = ed_p->next;
        free_ohci_ED(ed_p);
    }
}


// cppcheck-suppress misra-c2012-8.9 ; referenced by startup .c vector table
NVT_ITCM void OHCI_IRQHandler(void)
{
    TD_T       *td;
    TD_T       *td_prev;
    TD_T       *td_next;
    uint32_t   int_sts;

    int_sts = _ohci->HcInterruptStatus;

    //USB_debug("ohci int_sts = 0x%x\n", int_sts);

    if ((_ohci->HcInterruptEnable & USBH_HcInterruptEnable_SF_Msk) &&
            (int_sts & USBH_HcInterruptStatus_SF_Msk))
    {
        int_sts &= ~USBH_HcInterruptStatus_SF_Msk;

        _ohci->HcInterruptDisable = USBH_HcInterruptDisable_SF_Msk;
        (void)remove_ed();
        _ohci->HcInterruptStatus = USBH_HcInterruptStatus_SF_Msk;
    }

    if (int_sts & USBH_HcInterruptStatus_WDH_Msk)
    {
        //printf("!%02x\n", _ohci->HcFmNumber & 0xff);
        int_sts &= ~USBH_HcInterruptStatus_WDH_Msk;
        /*
         *  reverse done list
         */
        td = (TD_T *)(_hcca.done_head & TD_ADDR_MASK);
        _hcca.done_head = 0;
        td_prev = USBNULL;
        _ohci->HcInterruptStatus = USBH_HcInterruptStatus_WDH_Msk;

        while (td != USBNULL)
        {
            //TD_debug("Done list TD 0x%x => 0x%x\n", (int)td, (int)td->NextTD);
            td_next = (TD_T *)(td->NextTD & TD_ADDR_MASK);
            td->NextTD = (uint32_t)td_prev;
            td_prev = td;
            td = td_next;
        }

        td = td_prev;               /* first TD of the reversed done list */

        /*
         *  reclaim TDs
         */
        while (td != USBNULL)
        {
            TD_debug("Reclaim TD 0x%x, next 0x%x\n", (int)td, td->NextTD);
            td_next = (TD_T *)td->NextTD;
            td_done(td);
            free_ohci_TD(td);
            td = td_next;
        }
    }

    if (int_sts & USBH_HcInterruptStatus_RHSC_Msk)
    {
        _ohci->HcInterruptDisable = USBH_HcInterruptDisable_RHSC_Msk;
    }

    _ohci->HcInterruptStatus = int_sts;
    /* make sure that interrupt flag has been cleared. */
    int_sts = _ohci->HcInterruptStatus;
    (void)int_sts;
}

#ifdef DUMP_DESCRIPTOR

void dump_ohci1_int_table(void)
{
    int    i;

    for (i = 0; i < 32; i++)
    {
        USB_debug("%02d: ", i);

        ED_T   *ed;
        ed = (ED_T *)_hcca.int_table[i];

        while (ed != USBNULL)
        {
            USB_debug("0x%x (0x%x) => ", (int)ed, ed->HeadP);
            ed = (ED_T *)ed->NextED;
        }

        USB_debug("0\n");
    }
}

void dump_ohci1_regs(void)
{
    USB_debug("Dump OCHI registers: [0x%x]\n", _ohci->HcRevision);
    USB_debug("    HcRevision         = 0x%x\n", _ohci->HcRevision);
    USB_debug("    HcControl          = 0x%x\n", _ohci->HcControl);
    USB_debug("    HcCommandStatus    = 0x%x\n", _ohci->HcCommandStatus);
    USB_debug("    HcInterruptStatus  = 0x%x\n", _ohci->HcInterruptStatus);
    USB_debug("    HcInterruptEnable  = 0x%x\n", _ohci->HcInterruptEnable);
    USB_debug("    HcInterruptDisable = 0x%x\n", _ohci->HcInterruptDisable);
    USB_debug("    HcHCCA             = 0x%x\n", _ohci->HcHCCA);
    USB_debug("    HcPeriodCurrentED  = 0x%x\n", _ohci->HcPeriodCurrentED);
    USB_debug("    HcControlHeadED    = 0x%x\n", _ohci->HcControlHeadED);
    USB_debug("    HcControlCurrentED = 0x%x\n", _ohci->HcControlCurrentED);
    USB_debug("    HcBulkHeadED       = 0x%x\n", _ohci->HcBulkHeadED);
    USB_debug("    HcBulkCurrentED    = 0x%x\n", _ohci->HcBulkCurrentED);
    USB_debug("    HcDoneHead         = 0x%x\n", _ohci->HcDoneHead);
    USB_debug("    HcFmInterval       = 0x%x\n", _ohci->HcFmInterval);
    USB_debug("    HcFmRemaining      = 0x%x\n", _ohci->HcFmRemaining);
    USB_debug("    HcFmNumber         = 0x%x\n", _ohci->HcFmNumber);
    USB_debug("    HcPeriodicStart    = 0x%x\n", _ohci->HcPeriodicStart);
    USB_debug("    HcLSThreshold      = 0x%x\n", _ohci->HcLSThreshold);
    USB_debug("    HcRhDescriptorA    = 0x%x\n", _ohci->HcRhDescriptorA);
    USB_debug("    HcRhDescriptorB    = 0x%x\n", _ohci->HcRhDescriptorB);
    USB_debug("    HcRhStatus         = 0x%x\n", _ohci->HcRhStatus);
    USB_debug("    HcRhPortStatus0    = 0x%x\n", _ohci->HcRhPortStatus[0]);
}

void dump_ohci1_ports(void)
{
    USB_debug("_ohci port0=0x%x\n", _ohci->HcRhPortStatus[0]);
}

#endif  // DUMP_DESCRIPTOR

HC_DRV_T  ohci1_driver =
{
    ohci_init,               /* init               */
    ohci_shutdown,           /* shutdown           */
    ohci_suspend,            /* suspend            */
    ohci_resume,             /* resume             */
    ohci_ctrl_xfer,          /* ctrl_xfer          */
    ohci_bulk_xfer,          /* bulk_xfer          */
    ohci_int_xfer,           /* int_xfer           */
    ohci_iso_xfer,           /* iso_xfer           */
    ohci_quit_xfer,          /* quit_xfer          */
    ohci_rh_port_reset,      /* rthub_port_reset   */
    ohci_rh_polling          /* rthub_polling      */
};
#endif
/// @endcond HIDDEN_SYMBOLS