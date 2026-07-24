/**************************************************************************//**
 * @file     ehci.c
 * @version  V1.00
 * @brief    USB Host library EHCI (USB 2.0) host controller driver.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "hub.h"

/// @cond HIDDEN_SYMBOLS

#ifdef ENABLE_EHCI
#define EHCI_IRQHandler     HSUSBH_IRQHandler
extern NVT_ITCM void EHCI_IRQHandler(void);

//function list..
#define get_int_tree_head_node  ehci_get_int_tree_head_node

static QH_T   *_H_qh;                           /* head of reclamation list                   */
static qTD_T  *_ghost_qtd;                      /* used as a terminator qTD                   */
static QH_T *qh_remove_list;


#ifdef __ICCARM__
    #if (NVT_DCACHE_ON == 1)
        /* Periodic frame list are placed in a non-cacheable region */
        #pragma data_alignment=4096
        NVT_NONCACHEABLE uint32_t  _PFList[FL_SIZE];                 /* Periodic frame list (IAR)                  */
    #else
        #pragma data_alignment=4096
        uint32_t  _PFList[FL_SIZE];                 /* Periodic frame list (IAR)                  */
    #endif
#else
    #if (NVT_DCACHE_ON == 1)
        /* Periodic frame list are placed in a non-cacheable region */
        NVT_NONCACHEABLE uint32_t _PFList[FL_SIZE] __attribute__((aligned(4096)));  /* Periodic frame list        */
    #else
        uint32_t _PFList[FL_SIZE] __attribute__((aligned(4096)));  /* Periodic frame list        */
    #endif
#endif

static QH_T   *_Iqh[NUM_IQH];

#ifdef DUMP_DESCRIPTOR
void dump_ehci_regs(void)
{
    USB_debug("Dump HSUSBH(0x%x) registers:\n", _ehci->EHCVNR);
    USB_debug("    EHCVNR   = 0x%x\n", _ehci->EHCVNR);
    USB_debug("    EHCSPR   = 0x%x\n", _ehci->EHCSPR);
    USB_debug("    EHCCPR   = 0x%x\n", _ehci->EHCCPR);
    USB_debug("    UCMDR    = 0x%x\n", _ehci->UCMDR);
    USB_debug("    USTSR    = 0x%x\n", _ehci->USTSR);
    USB_debug("    UIENR    = 0x%x\n", _ehci->UIENR);
    USB_debug("    UFINDR   = 0x%x\n", _ehci->UFINDR);
    USB_debug("    UPFLBAR  = 0x%x\n", _ehci->UPFLBAR);
    USB_debug("    UCALAR   = 0x%x\n", _ehci->UCALAR);
    //    USB_debug("    UASSTR   = 0x%x\n", _ehci->UASSTR);
    USB_debug("    UCFGR    = 0x%x\n", _ehci->UCFGR);
    USB_debug("    UPSCR0   = 0x%x\n", _ehci->UPSCR[0]);
    //    USB_debug("    PHYCTL0  = 0x%x\n", _ehci->USBPCR0);
    //    USB_debug("    PHYCTL1  = 0x%x\n", _ehci->USBPCR1);
}

void dump_ehci_ports()
{
    USB_debug("_ehci port0=0x%x\n", _ehci->UPSCR[0]);
}

void dump_ehci_qtd(qTD_T *qtd)
{
    USB_debug("    [qTD] - 0x%08x\n", (int)qtd);
    USB_debug("        0x%08x (Next qtd Pointer)\n", qtd->Next_qTD);
    USB_debug("        0x%08x (Alternate Next qtd Pointer)\n", qtd->Alt_Next_qTD);
    USB_debug("        0x%08x (qtd Token) PID: %s, Bytes: %d, IOC: %d\n", qtd->Token, (((qtd->Token >> 8) & 0x3) == 0) ? "OUT" : ((((qtd->Token >> 8) & 0x3) == 1) ? "IN" : "SETUP"),
              (qtd->Token >> 16) & 0x7FFF, (qtd->Token >> 15) & 0x1);
    USB_debug("        0x%08x (Buffer Pointer (page 0))\n", qtd->Bptr[0]);
    //USB_debug("        0x%08x (Buffer Pointer (page 1))\n", qtd->Bptr[1]);
    //USB_debug("        0x%08x (Buffer Pointer (page 2))\n", qtd->Bptr[2]);
    //USB_debug("        0x%08x (Buffer Pointer (page 3))\n", qtd->Bptr[3]);
    //USB_debug("        0x%08x (Buffer Pointer (page 4))\n", qtd->Bptr[4]);
    USB_debug("\n");
}

void dump_ehci_asynclist(void)
{
    QH_T     *qh = _H_qh;
    qTD_T    *qtd;

    USB_debug(">>> Dump EHCI Asynchronous List <<<\n");

    do
    {
        USB_debug("[QH] - 0x%08x\n", (int)qh);
        USB_debug("    0x%08x (Queue Head Horizontal Link Pointer, Queue Head DWord 0)\n", qh->HLink);
        USB_debug("    0x%08x (Endpoint Characteristics) DevAddr: %d, EP: 0x%x, PktSz: %d, Speed: %s\n", qh->Chrst, qh->Chrst & 0x7F, (qh->Chrst >> 8) & 0xF, (qh->Chrst >> 16) & 0x7FF,
                  (((qh->Chrst >> 12) & 0x3) == 0) ? "Full" : ((((qh->Chrst >> 12) & 0x3) == 1) ? "Low" : "High"));
        USB_debug("    0x%08x (Endpoint Capabilities: Queue Head DWord 2)\n", qh->Cap);
        USB_debug("    0x%08x (Current qtd Pointer)\n", qh->Curr_qTD);
        USB_debug("    --- Overlay Area ---\n");
        USB_debug("    0x%08x (Next qtd Pointer)\n", qh->OL_Next_qTD);
        USB_debug("    0x%08x (Alternate Next qtd Pointer)\n", qh->OL_Alt_Next_qTD);
        USB_debug("    0x%08x (qtd Token)\n", qh->OL_Token);
        USB_debug("    0x%08x (Buffer Pointer (page 0))\n", qh->OL_Bptr[0]);
        USB_debug("\n");

        qtd = QTD_PTR(qh->Curr_qTD);

        while (qtd != USBNULL)
        {
            dump_ehci_qtd(qtd);
            qtd = QTD_PTR(qtd->Next_qTD);
        }

        qh = QH_PTR(qh->HLink);
    } while (qh != _H_qh);
}

static void dump_ehci_asynclist_simple(void)
{
    QH_T     *qh = _H_qh;

    USB_debug(">>> EHCI Asynchronous List <<<\n");
    USB_debug("[QH] => ");

    do
    {
        USB_debug("0x%08x ", (int)qh);
        qh = QH_PTR(qh->HLink);
    } while (qh != _H_qh);

    USB_debug("\n");
}

void dump_ehci_period_frame_list_simple(void)
{
    QH_T     *qh = _Iqh[NUM_IQH - 1U];

    USB_debug(">>> EHCI period frame list simple <<<\n");
    USB_debug("[FList] => ");

    do
    {
        USB_debug("0x%08x ", (int)qh);
        qh = QH_PTR(qh->HLink);
    } while (qh != USBNULL);

    USB_debug("\n");
}

static void dump_ehci_period_frame_list()
{
    int    i;
    QH_T   *qh;

    for (i = 0; i < FL_SIZE; i++)
    {
        USB_debug("!%02d: ", i);
        qh = QH_PTR(_PFList[i]);;

        while (qh != USBNULL)
        {
            // USB_debug("0x%x (0x%x) => ", (int)qh, qh->HLink);
            USB_debug("0x%x => ", (int)qh);
            qh = QH_PTR(qh->HLink);
        }

        USB_debug("0\n");
    }
}

#endif  /* ENABLE_ERROR_MSG */

static void init_periodic_frame_list(void)
{
    uint32_t i;
    uint32_t idx;


    (void)memset(_PFList, 0, sizeof(_PFList));

    iso_ep_list = USBNULL;

    for (i = NUM_IQH; i > 0U; i--)            /* interval = i^2                             */
    {
        uint32_t iqh_idx;

        iqh_idx = i - 1U;

        _Iqh[iqh_idx] = alloc_ehci_QH();

        _Iqh[iqh_idx]->HLink           = QH_HLNK_END;
        _Iqh[iqh_idx]->Curr_qTD        = (uint32_t)_ghost_qtd;
        _Iqh[iqh_idx]->OL_Next_qTD     = QTD_LIST_END;
        _Iqh[iqh_idx]->OL_Alt_Next_qTD = (uint32_t)_ghost_qtd;
        _Iqh[iqh_idx]->OL_Token        = QTD_STS_HALT;

        uint32_t interval;
        interval = (0x1UL << (iqh_idx));

        for (idx = (interval - 1U); idx < (uint32_t)FL_SIZE; idx += interval)
        {
            if (_PFList[idx] == 0U)         /* is empty list, insert directly             */
            {
                _PFList[idx] = QH_HLNK_QH(_Iqh[iqh_idx]);
            }
            else
            {
                QH_T   *qh_p;
                qh_p = QH_PTR(_PFList[idx]);

                while (1)
                {
                    /* stop when already chained by previous visit, or reached end of list */
                    if ((qh_p == _Iqh[iqh_idx]) || (qh_p->HLink == QH_HLNK_END))
                    {
                        if (qh_p != _Iqh[iqh_idx])         /* reached end, not yet chained */
                        {
                            qh_p->HLink = QH_HLNK_QH(_Iqh[iqh_idx]);
                        }

                        break;
                    }

                    qh_p = QH_PTR(qh_p->HLink);
                }
            }
        }
    }
}

static QH_T *get_int_tree_head_node(uint32_t interval)
{
    uint32_t i;
    uint32_t interval_local;

    interval_local = interval / 8U;          /* each frame list entry for 8 micro-frame    */

    for (i = 0U; i < (NUM_IQH - 1U); i++)
    {
        interval_local >>= 1;

        if (interval_local == 0U)
        {
            return _Iqh[i];
        }
    }

    return _Iqh[NUM_IQH - 1U];
}

static uint32_t make_int_s_mask(uint32_t bInterval)
{
    uint32_t order;
    uint32_t interval;
    uint32_t interval_local;

    interval = 1U;
    interval_local = bInterval;

    while (interval_local > 1U)
    {
        interval *= 2U;
        interval_local--;
    }

    if (interval < 2U)
    {
        return 0xFFU;                       /* interval 1                                 */
    }

    if (interval < 4U)
    {
        return 0x55U;                       /* interval 2                                 */
    }

    if (interval < 8U)
    {
        return 0x22U;                       /* interval 4                                 */
    }

    order = 0U;

    while (interval > 1U)
    {
        interval >>= 1U;
        order++;
    }

    return (0x1U << (order % 8U));
}

static int  ehci_init(void)
{
    uint32_t timeout = 250U * 1000U;        /* EHCI reset time-out 250 ms                */

    /*------------------------------------------------------------------------------------*/
    /*  Reset EHCI host controller                                                        */
    /*------------------------------------------------------------------------------------*/
    _ehci->UCMDR = HSUSBH_UCMDR_HCRST_Msk;

    while (((_ehci->UCMDR & HSUSBH_UCMDR_HCRST_Msk) != 0U) && (timeout > 0U))
    {
        delay_us(1000);
        timeout -= 1000U;
    }

    if ((_ehci->UCMDR & HSUSBH_UCMDR_HCRST_Msk) != 0U)
    {
        return USBH_ERR_EHCI_INIT;
    }

    _ehci->UCMDR = UCMDR_INT_THR_CTRL | HSUSBH_UCMDR_RUN_Msk;

    _ghost_qtd = alloc_ehci_qTD(USBNULL);
    //_ghost_qtd->Token = 0x11197B3F;    //QTD_STS_HALT;  visit_qtd() will not remove a qTD with this mark. It represents a qhost qTD.
    _ghost_qtd->Token = 0x11197B7FU;
    /*------------------------------------------------------------------------------------*/
    /*  Initialize asynchronous list                                                      */
    /*------------------------------------------------------------------------------------*/
    qh_remove_list = USBNULL;

    /* Create the QH list head with H-bit 1 */
    _H_qh = alloc_ehci_QH();
    _H_qh->HLink   = QH_HLNK_QH(_H_qh);     /* circular link to itself, the only one QH   */
    _H_qh->Chrst   = QH_RCLM_LIST_HEAD;     /* it's the head of reclamation list          */
    _H_qh->Curr_qTD        = (uint32_t)_ghost_qtd;
    _H_qh->OL_Next_qTD     = QTD_LIST_END;
    _H_qh->OL_Alt_Next_qTD = (uint32_t)_ghost_qtd;
    _H_qh->OL_Token        = QTD_STS_HALT;
    _ehci->UCALAR = (uint32_t)_H_qh;
    //USB_debug("_ehci->UCALAR - 0x%x \n", _ehci->UCALAR);

    /*------------------------------------------------------------------------------------*/
    /*  Initialize periodic list                                                          */
    /*------------------------------------------------------------------------------------*/
    if (FL_SIZE == 256U)
    {
        _ehci->UCMDR |= (0x2U << HSUSBH_UCMDR_FLSZ_Pos);
    }
    else if (FL_SIZE == 512U)
    {
        _ehci->UCMDR |= (0x1U << HSUSBH_UCMDR_FLSZ_Pos);
    }
    else if (FL_SIZE == 1024U)
    {
        _ehci->UCMDR |= (0x0U << HSUSBH_UCMDR_FLSZ_Pos);
    }
    else
    {
        return USBH_ERR_EHCI_INIT;               /* Invalid FL_SIZE setting!              */
    }

    _ehci->UPFLBAR = (uint32_t)_PFList;
    //USB_debug("_ehci->UPFLBAR - 0x%x \n", _ehci->UPFLBAR);
    /*------------------------------------------------------------------------------------*/
    /*  start run                                                                         */
    /*------------------------------------------------------------------------------------*/

    _ehci->UCFGR = 0x1U;                         /* enable port routing to EHCI           */
    _ehci->UIENR = HSUSBH_UIENR_USBIEN_Msk | HSUSBH_UIENR_UERRIEN_Msk | HSUSBH_UIENR_HSERREN_Msk | HSUSBH_UIENR_IAAEN_Msk;

    delay_us(1000);                              /* delay 1 ms                            */

    _ehci->UPSCR[0] = HSUSBH_UPSCR_PP_Msk;      /* enable port 1 port power               */
    //_ehci->UPSCR[1] = HSUSBH_UPSCR_PP_Msk | HSUSBH_UPSCR_PO_Msk;     set port 2 owner to OHCI

    init_periodic_frame_list();

    delay_us(10 * 1000);                        /* delay 10 ms                            */

    return 0;
}

static int  ehci_reinit(void)
{
    int      timeout = 250 * 1000;          /* EHCI reset time-out 250 ms                */
    HSUSBH_T ehci_backup = { 0 };

    (void)memcpy(&ehci_backup, _ehci, sizeof(ehci_backup));

    /*------------------------------------------------------------------------------------*/
    /*  Reset EHCI host controller                                                        */
    /*------------------------------------------------------------------------------------*/
    _ehci->UCMDR = HSUSBH_UCMDR_HCRST_Msk;

    while ((_ehci->UCMDR & HSUSBH_UCMDR_HCRST_Msk) && (timeout > 0))
    {
        delay_us(1000);
        timeout -= 1000;
    }

    if (_ehci->UCMDR & HSUSBH_UCMDR_HCRST_Msk)
    {
        return USBH_ERR_EHCI_INIT;
    }

    _ehci->UCMDR = ehci_backup.UCMDR & ~HSUSBH_UCMDR_IAAD_Msk;;

    _ehci->UCALAR = ehci_backup.UCALAR;

    _ehci->UPFLBAR = ehci_backup.UPFLBAR;

    /*------------------------------------------------------------------------------------*/
    /*  start run                                                                         */
    /*------------------------------------------------------------------------------------*/

    _ehci->UCFGR = 0x1U;                          /* enable port routing to EHCI           */
    _ehci->UIENR = HSUSBH_UIENR_USBIEN_Msk | HSUSBH_UIENR_UERRIEN_Msk | HSUSBH_UIENR_HSERREN_Msk | HSUSBH_UIENR_IAAEN_Msk;

    delay_us(1000);                              /* delay 1 ms                            */

    _ehci->UPSCR[0] = HSUSBH_UPSCR_PP_Msk;      /* enable port 1 port power               */

    delay_us(10 * 1000);                        /* delay 10 ms                            */

    return 0;
}

static void ehci_suspend(void)
{
    if (_ehci->UPSCR[0] & HSUSBH_UPSCR_PE_Msk)
    {
        _ehci->UPSCR[0] |= HSUSBH_UPSCR_SUSPEND_Msk;
    }
}

static void ehci_resume(void)
{
    if (_ehci->UPSCR[0] & HSUSBH_UPSCR_PE_Msk)
    {
        _ehci->UPSCR[0] |= HSUSBH_UPSCR_FPR_Msk;
        delay_us(25000);                         /* keep resume signal for 20 ms */
        _ehci->UPSCR[0] &= ~HSUSBH_UPSCR_FPR_Msk;
    }

    delay_us(1000);
}

static void ehci_shutdown(void)
{
    ehci_suspend();
}

static void move_qh_to_remove_list(QH_T *qh)
{
    QH_T       *q;

    // USB_debug("move_qh_to_remove_list - 0x%x (0x%x)\n", (int)qh, qh->Chrst);

    /* check if this ED found in ed_remove_list */
    q = qh_remove_list;

    while (q)
    {
        if (q == qh)                        /* This QH found in qh_remove_list.           */
        {
            return;                         /* Do nothing, return...                      */
        }

        q = q->next;
    }

    /*------------------------------------------------------------------------------------*/
    /*  deactive the QH                                                                   */
    /*------------------------------------------------------------------------------------*/
    qh->Chrst |= QH_RCLM_LIST_HEAD;
    qh->OL_Token = (qh->OL_Token & (~QTD_STS_ACTIVE)) | QTD_STS_HALT;

    /*------------------------------------------------------------------------------------*/
    /*  Search asynchronous frame list and remove qh if found in list.                    */
    /*------------------------------------------------------------------------------------*/
    q = _H_qh;                              /* find and remove it from asynchronous list  */

    while (QH_PTR(q->HLink) != _H_qh)
    {
        if (QH_PTR(q->HLink) == qh)
        {
            /* q's next QH is qh, found...           */
            q->HLink = qh->HLink;                /* remove qh from list                   */

            qh->next = qh_remove_list;           /* add qh to qh_remove_list              */
            qh_remove_list = qh;

            return;                              /* done                                  */
        }

        q = QH_PTR(q->HLink);               /* advance to next QH in asynchronous list    */
    }

    /*------------------------------------------------------------------------------------*/
    /*  Search periodic frame list and remove qh if found in list.                        */
    /*------------------------------------------------------------------------------------*/
    q =  _Iqh[NUM_IQH - 1U];

    while (q->HLink != QH_HLNK_END)
    {
        if (QH_PTR(q->HLink) == qh)
        {
            /* q's next QH is qh, found...           */
            q->HLink = qh->HLink;                /* remove qh from list                   */

            qh->next = qh_remove_list;           /* add qh to qh_remove_list              */
            qh_remove_list = qh;

            return;                              /* done                                  */
        }

        q = QH_PTR(q->HLink);               /* advance to next QH in asynchronous list    */
    }

    ENABLE_EHCI_IRQ();
}

static void remove_queue_head(QH_T *qh)
{
    DISABLE_EHCI_IRQ();
    move_qh_to_remove_list(qh);
    ENABLE_EHCI_IRQ();
    _ehci->UCMDR |= HSUSBH_UCMDR_IAAD_Msk;
}

static void append_to_qtd_list_of_QH(QH_T *qh, qTD_T *qtd)
{
    qTD_T  *q;

    if (qh->qtd_list == USBNULL)
    {
        qh->qtd_list = qtd;
    }
    else
    {
        q = qh->qtd_list;

        while (q->next != USBNULL)
        {
            q = q->next;
        }

        q->next = qtd;
    }
}

/*
 *  If ep==USBNULL, it's a control endpoint QH.
 */
static void  write_qh(UDEV_T *udev, const EP_INFO_T *ep, QH_T *qh)
{
    uint32_t   chrst;
    uint32_t   cap;

    /*------------------------------------------------------------------------------------*/
    /*  Write QH DWord 1 - Endpoint Characteristics                                       */
    /*------------------------------------------------------------------------------------*/
    if (ep == USBNULL)                             /* is control endpoint?                   */
    {
        if (udev->descriptor.bMaxPacketSize0 == 0U)
        {
            if (udev->speed == SPEED_LOW)       /* give a default maximum packet size     */
            {
                udev->descriptor.bMaxPacketSize0 = 8U;
            }
            else
            {
                udev->descriptor.bMaxPacketSize0 = 64U;
            }
        }

        chrst = QH_DTC | QH_NAK_RL | ((uint32_t)udev->descriptor.bMaxPacketSize0 << 16U);

        if (udev->speed != SPEED_HIGH)
        {
            chrst |= QH_CTRL_EP_FLAG;           /* non-high-speed control endpoint        */
        }
    }
    else                                        /* not a control endpoint                 */
    {
        chrst = QH_NAK_RL | ((uint32_t)ep->wMaxPacketSize << 16U);
        chrst |= ((uint32_t)(ep->bEndpointAddress & 0x0FU) << 8U);      /* Endpoint Address             */
    }

    if (udev->speed == SPEED_LOW)
    {
        chrst |= QH_EPS_LOW;
    }
    else if (udev->speed == SPEED_FULL)
    {
        chrst |= QH_EPS_FULL;
    }
    else
    {
        chrst |= QH_EPS_HIGH;
    }

    chrst |= (uint32_t)udev->dev_num;

    qh->Chrst = chrst;

    /*------------------------------------------------------------------------------------*/
    /*  Write QH DWord 2 - Endpoint Capabilities                                         */
    /*------------------------------------------------------------------------------------*/
    if (udev->speed == SPEED_HIGH)
    {
        cap = 0x40000000U;
    }
    else
    {
        /*
         *  Backtrace device tree until the USB 2.0 hub found
         */
        HUB_DEV_T   *hub;
        uint32_t    port_num;

        port_num = udev->port_num;
        hub = udev->parent;

        while ((hub != USBNULL) && (hub->iface->udev->speed != SPEED_HIGH))
        {
            port_num = hub->iface->udev->port_num;
            hub = hub->iface->udev->parent;
        }

        if (hub != USBNULL)
        {
            cap = ((uint32_t)port_num << QH_HUB_PORT_Pos) |
                  ((uint32_t)hub->iface->udev->dev_num << QH_HUB_ADDR_Pos);
        }
        else
        {
            cap = ((uint32_t)port_num << QH_HUB_PORT_Pos);
        }
    }

    qh->Cap = cap;
}

static void  write_qtd_bptr(qTD_T *qtd, uint32_t buff_addr, int xfer_len)
{
    int       i;
    uint32_t  next_buff_addr;

    qtd->xfer_len = xfer_len;
    qtd->Bptr[0] = buff_addr;

    next_buff_addr = (buff_addr + 0x1000U) & ~0xFFFU;

    for (i = 1; i < 5; i++)
    {
        qtd->Bptr[i] = next_buff_addr;
        next_buff_addr += 0x1000U;
    }
}

static int ehci_ctrl_xfer(UTR_T *utr)
{
    UDEV_T     *udev;
    QH_T       *qh;
    qTD_T      *qtd_setup;
    qTD_T      *qtd_data;
    qTD_T      *qtd_status;
    uint32_t   token;
    int        is_new_qh = 0;

    udev = utr->udev;

    if (utr->data_len > 0U)
    {
        if ((((uint32_t)utr->buff + (uint32_t)utr->data_len) > ((((uint32_t)utr->buff) & ~0xFFFU) + 0x5000U)))
        {
            return USBH_ERR_BUFF_OVERRUN;
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  Allocate and link QH                                                              */
    /*------------------------------------------------------------------------------------*/
    if (udev->ep0.hw_pipe != USBNULL)
    {
        qh = (QH_T *)udev->ep0.hw_pipe;

        if (qh->qtd_list != USBNULL)
        {
            return USBH_ERR_EHCI_QH_BUSY;
        }
    }
    else
    {
        qh = alloc_ehci_QH();

        if (qh == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }

        udev->ep0.hw_pipe = (void *)qh;     /* driver can find QH from EP                 */
        is_new_qh = 1;
    }

    write_qh(udev, USBNULL, qh);
    utr->ep = &udev->ep0;                   /* driver can find EP from UTR                */

    /*------------------------------------------------------------------------------------*/
    /*  Allocate qTDs                                                                     */
    /*------------------------------------------------------------------------------------*/
    qtd_setup = alloc_ehci_qTD(utr);        /* allocate qTD for SETUP                     */

    if (utr->data_len > 0U)
    {
        qtd_data = alloc_ehci_qTD(utr);     /* allocate qTD for DATA                      */
    }
    else
    {
        qtd_data = USBNULL;
    }

    qtd_status = alloc_ehci_qTD(utr);       /* allocate qTD for USTSR                     */

    if (qtd_status == USBNULL)                 /* out of memory?                             */
    {
        if (qtd_setup != USBNULL)
        {
            free_ehci_qTD(qtd_setup);       /* free memory                                */
        }

        if (qtd_data != USBNULL)
        {
            free_ehci_qTD(qtd_data);        /* free memory                                */
        }

        return USBH_ERR_MEMORY_OUT;         /* out of memory                              */
    }

    // USB_debug("qh=0x%x, qtd_setup=0x%x, qtd_data=0x%x, qtd_status=0x%x\n", (int)qh, (int)qtd_setup, (int)qtd_data, (int)qtd_status);

    /*------------------------------------------------------------------------------------*/
    /* prepare SETUP stage qTD                                                            */
    /*------------------------------------------------------------------------------------*/
    qtd_setup->qh = qh;
    //qtd_setup->utr = utr;
    write_qtd_bptr(qtd_setup, (uint32_t)&utr->setup, 8);
    append_to_qtd_list_of_QH(qh, qtd_setup);
    qtd_setup->Token = (8UL << 16) | QTD_ERR_COUNTER | QTD_PID_SETUP | QTD_STS_ACTIVE;

    /*------------------------------------------------------------------------------------*/
    /* prepare DATA stage qTD                                                             */
    /*------------------------------------------------------------------------------------*/
    if (utr->data_len > 0U)
    {
        qtd_setup->Next_qTD = (uint32_t)qtd_data;
        qtd_data->Next_qTD = (uint32_t)qtd_status;

        if ((utr->setup.bmRequestType & 0x80U) == REQ_TYPE_OUT)
        {
            token = QTD_ERR_COUNTER | QTD_PID_OUT | QTD_STS_ACTIVE;
        }
        else
        {
            token = QTD_ERR_COUNTER | QTD_PID_IN | QTD_STS_ACTIVE;
        }

        qtd_data->qh = qh;
        //qtd_data->utr = utr;
        write_qtd_bptr(qtd_data, (uint32_t)utr->buff, utr->data_len);
        append_to_qtd_list_of_QH(qh, qtd_data);
        qtd_data->Token = QTD_DT | ((uint32_t)utr->data_len << 16U) | token;
    }
    else
    {
        qtd_setup->Next_qTD = (uint32_t)qtd_status;
    }

    /*------------------------------------------------------------------------------------*/
    /* prepare USTSR stage qTD                                                            */
    /*------------------------------------------------------------------------------------*/
    qtd_status->Next_qTD = (uint32_t)_ghost_qtd;
    qtd_status->Alt_Next_qTD = (uint32_t)QTD_LIST_END;

    if ((utr->setup.bmRequestType & 0x80U) == REQ_TYPE_OUT)
    {
        token = QTD_ERR_COUNTER | QTD_PID_IN | QTD_STS_ACTIVE;
    }
    else
    {
        token = QTD_ERR_COUNTER | QTD_PID_OUT | QTD_STS_ACTIVE;
    }

    qtd_status->qh = qh;
    //qtd_status->utr = utr;
    append_to_qtd_list_of_QH(qh, qtd_status);
    qtd_status->Token = QTD_DT | QTD_IOC | token;

    /*------------------------------------------------------------------------------------*/
    /* Update QH overlay                                                                  */
    /*------------------------------------------------------------------------------------*/
    qh->Curr_qTD = 0U;
    qh->OL_Next_qTD = (uint32_t)qtd_setup;
    qh->OL_Alt_Next_qTD = QTD_LIST_END;
    qh->OL_Token = 0U;

    /*------------------------------------------------------------------------------------*/
    /* Link QH and start asynchronous transfer                                            */
    /*------------------------------------------------------------------------------------*/
    if (is_new_qh)
    {
        qh->HLink = _H_qh->HLink;
        _H_qh->HLink = QH_HLNK_QH(qh);
    }

    /*  Start transfer */
    _ehci->UCMDR |= HSUSBH_UCMDR_ASEN_Msk;      /* start asynchronous transfer            */
    return 0;
}

static int ehci_bulk_xfer(UTR_T *utr)
{
    UDEV_T     *udev;
    EP_INFO_T  *ep = utr->ep;
    QH_T       *qh;
    qTD_T      *qtd;
    qTD_T      *qtd_pre;
    uint32_t   data_len;
    uint32_t   xfer_len;
    uint8_t    *buff;
    uint32_t   buff_offset;
    uint32_t   token;
    int        is_new_qh = 0;

    //USB_debug("Bulk XFER =>\n");
    // dump_ehci_asynclist_simple();

    udev = utr->udev;

    if (ep->hw_pipe != USBNULL)
    {
        qh = (QH_T *)ep->hw_pipe ;

        if (qh->qtd_list != USBNULL)
        {
            return USBH_ERR_EHCI_QH_BUSY;
        }
    }
    else
    {
        qh = alloc_ehci_QH();

        if (qh == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }

        is_new_qh = 1;
        write_qh(udev, ep, qh);
        ep->hw_pipe = (void *)qh;           /* associate QH with endpoint                 */
    }

    /*------------------------------------------------------------------------------------*/
    /* Prepare qTDs                                                                       */
    /*------------------------------------------------------------------------------------*/
    data_len = utr->data_len;
    buff = utr->buff;
    buff_offset = 0U;
    qtd_pre = USBNULL;

    while (data_len > 0U)
    {
        qtd = alloc_ehci_qTD(utr);

        if (qtd == USBNULL)                    /* failed to allocate a qTD                   */
        {
            qtd = qh->qtd_list;

            while (qtd != USBNULL)
            {
                qtd_pre = qtd;
                qtd = qtd->next;
                free_ehci_qTD(qtd_pre);
            }

            if (is_new_qh)
            {
                free_ehci_QH(qh);
                ep->hw_pipe = USBNULL;
            }

            return USBH_ERR_MEMORY_OUT;
        }

        if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT)
        {
            token = QTD_ERR_COUNTER | QTD_PID_OUT | QTD_STS_ACTIVE;
        }
        else
        {
            token = QTD_ERR_COUNTER | QTD_PID_IN | QTD_STS_ACTIVE;
        }

        if (data_len > 0x4000U)             /* force maximum x'fer length 16K per qTD     */
        {
            xfer_len = 0x4000U;
        }
        else
        {
            xfer_len = data_len;            /* remaining data length < 4K                 */
        }

        qtd->qh = qh;
        qtd->Next_qTD = (uint32_t)_ghost_qtd;
        qtd->Alt_Next_qTD = QTD_LIST_END; //(uint32_t)_ghost_qtd;
        write_qtd_bptr(qtd, (uint32_t)&buff[buff_offset], (int)xfer_len);
        append_to_qtd_list_of_QH(qh, qtd);
        qtd->Token = (xfer_len << 16U) | token;

        buff_offset += xfer_len;            /* advanced buffer offset                     */
        data_len -= xfer_len;

        if (data_len == 0U)                  /* is this the latest qTD?                   */
        {
            qtd->Token |= QTD_IOC;          /* ask to raise an interrupt on the last qTD  */
            qtd->Next_qTD = (uint32_t)_ghost_qtd;     /* qTD list end                     */
        }

        if (qtd_pre != USBNULL)
        {
            qtd_pre->Next_qTD = (uint32_t)qtd;
        }

        qtd_pre = qtd;
    }

    //USB_debug("utr=0x%x, qh=0x%x, qtd=0x%x\n", (int)utr, (int)qh, (int)qh->qtd_list);

    qtd = qh->qtd_list;

    //    qh->Curr_qTD = 0; //(uint32_t)qtd;
    qh->OL_Next_qTD = (uint32_t)qtd;
    //  qh->OL_Alt_Next_qTD = QTD_LIST_END;

    /*------------------------------------------------------------------------------------*/
    /* Link QH and start asynchronous transfer                                            */
    /*------------------------------------------------------------------------------------*/
    if (is_new_qh)
    {
        (void)memcpy(&(qh->OL_Bptr[0]), &(qtd->Bptr[0]), 20);
        qh->Curr_qTD = (uint32_t)qtd;

        qh->OL_Token = 0; //qtd->Token;

        if (utr->ep->bToggle != 0U)
        {
            qh->OL_Token |= QTD_DT;
        }

        qh->HLink = _H_qh->HLink;
        _H_qh->HLink = QH_HLNK_QH(qh);
    }

    /*  Start transfer */
    _ehci->UCMDR |= HSUSBH_UCMDR_ASEN_Msk;      /* start asynchronous transfer            */

    return 0;
}

static int ehci_int_xfer(UTR_T *utr)
{
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    QH_T       *qh;
    QH_T       *iqh;
    qTD_T      *qtd;
    qTD_T      *dummy_qtd;
    uint32_t   token;

    dummy_qtd = alloc_ehci_qTD(USBNULL);     /* allocate a new dummy qTD                    */

    if (dummy_qtd == USBNULL)
    {
        return USBH_ERR_MEMORY_OUT;
    }

    dummy_qtd->Token &= ~(QTD_STS_ACTIVE | QTD_STS_HALT);

    if (ep->hw_pipe != USBNULL)
    {
        qh = (QH_T *)ep->hw_pipe ;
    }
    else
    {
        qh = alloc_ehci_QH();

        if (qh == USBNULL)
        {
            free_ehci_qTD(dummy_qtd);
            return USBH_ERR_MEMORY_OUT;
        }

        write_qh(udev, ep, qh);
        qh->Chrst &= ~0xF0000000U;

        if (udev->speed == SPEED_HIGH)
        {
            qh->Cap = (0x1UL << QH_MULT_Pos) | (qh->Cap & 0xFFU) | make_int_s_mask(ep->bInterval);
        }
        else
        {
            qh->Cap = (0x1UL << QH_MULT_Pos) | (qh->Cap & ~(QH_C_MASK_Msk | QH_S_MASK_Msk)) | 0x7802U;
        }

        ep->hw_pipe = (void *)qh;           /* associate QH with endpoint                 */

        /*
         *  Allocate another dummy qTD
         */
        qtd = alloc_ehci_qTD(USBNULL);    /* allocate a new dummy qTD                   */

        if (qtd == USBNULL)
        {
            free_ehci_qTD(dummy_qtd);
            free_ehci_QH(qh);
            return USBH_ERR_MEMORY_OUT;
        }

        qtd->Token &= ~((uint32_t)QTD_STS_ACTIVE | (uint32_t)QTD_STS_HALT);

        qh->dummy = dummy_qtd;
        qh->OL_Next_qTD = (uint32_t)dummy_qtd;
        qh->OL_Token = 0U;    /* !Active & !Halted */

        /*
         *  link QH
         */
        if (udev->speed == SPEED_HIGH)      /* get head node of this interval             */
        {
            iqh = get_int_tree_head_node(ep->bInterval);
        }
        else
        {
            iqh = get_int_tree_head_node((uint32_t)ep->bInterval * 8U);
        }

        qh->HLink = iqh->HLink;             /* Add to list of the same interval           */
        iqh->HLink = QH_HLNK_QH(qh);

        dummy_qtd = qtd;
    }

    qtd = qh->dummy;                        /* use the current dummy qTD                  */
    qtd->Next_qTD = (uint32_t)dummy_qtd;
    qtd->utr = utr;
    qh->dummy = dummy_qtd;                  /* give the new dummy qTD                     */

    /*------------------------------------------------------------------------------------*/
    /*  Prepare qTD                                                                       */
    /*------------------------------------------------------------------------------------*/

    if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT)
    {
        token = QTD_ERR_COUNTER | QTD_PID_OUT;
    }
    else
    {
        token = QTD_ERR_COUNTER | QTD_PID_IN;
    }

    qtd->qh = qh;
    qtd->Alt_Next_qTD = (uint32_t)QTD_LIST_END;
    write_qtd_bptr(qtd, (uint32_t)utr->buff, utr->data_len);
    append_to_qtd_list_of_QH(qh, qtd);
    qtd->Token = QTD_IOC | ((uint32_t)utr->data_len << 16U) | token | QTD_STS_ACTIVE;

    // printf("ehci_int_xfer - qh: 0x%x, 0x%x, 0x%x\n", (int)qh, (int)qh->Chrst, (int)qh->Cap);

    _ehci->UCMDR |= HSUSBH_UCMDR_PSEN_Msk;      /* periodic list enable                   */
    return 0;
}

/*
 *  Quit current trasnfer via UTR or hardware EP.
 */
static int ehci_quit_xfer(UTR_T *utr, EP_INFO_T *ep)
{
    QH_T       *qh;
    uint32_t t0;

    // USB_debug("ehci_quit_xfer - utr: 0x%x, ep: 0x%x\n", (int)utr, (int)ep);

    DISABLE_EHCI_IRQ();

    if (ehci_quit_iso_xfer(utr, ep) == 0)
    {
        ENABLE_EHCI_IRQ();
        return 0;
    }

    ENABLE_EHCI_IRQ();

    if (utr != USBNULL)
    {
        if (utr->ep == USBNULL)
        {
            return USBH_ERR_NOT_FOUND;
        }

        qh = (QH_T *)(utr->ep->hw_pipe);

        if (qh == USBNULL)
        {
            return USBH_ERR_NOT_FOUND;
        }

        /* add the QH to remove list, it will be removed on the next IAAD interrupt       */
        remove_queue_head(qh);
        utr->ep->hw_pipe = USBNULL;
    }

    if ((ep != USBNULL) && (ep->hw_pipe != USBNULL))
    {
        qh = (QH_T *)(ep->hw_pipe);
        /* add the QH to remove list, it will be removed on the next IAAD interrupt       */
        remove_queue_head(qh);
        ep->hw_pipe = USBNULL;
    }

    /* wait until IAAD interrupt processed */
    t0 = get_ticks();

    while (((_ehci->UCMDR & HSUSBH_UCMDR_IAAD_Msk) != 0U) || ((_ehci->USTSR & HSUSBH_USTSR_IAA_Msk) != 0U))
    {
        if ((get_ticks() - t0) > 2U)
        {
            USB_error("%s - IAAD lost!!  UCMDR:0x%x, USTSR: 0x%x, UIENR: 0x%x\n", __func__, _ehci->UCMDR, _ehci->USTSR, _ehci->UIENR);
            (void)ehci_reinit();
            _ehci->UCMDR |= HSUSBH_UCMDR_IAAD_Msk;
            break;
        }
    }

    return 0;
}

static int visit_qtd(qTD_T *qtd)
{
    if ((qtd->Token == 0x11197B3FU) || (qtd->Token == 0x1197B3FU))
    {
        return 0;                    /* A Dummy qTD or qTD on writing, don't touch it.    */
    }

    // USB_debug("Visit qtd 0x%x - 0x%x\n", (int)qtd, qtd->Token);

    if ((qtd->Token & QTD_STS_ACTIVE) == 0U)
    {
        if ((qtd->Token & ((uint32_t)QTD_STS_HALT | (uint32_t)QTD_STS_DATA_BUFF_ERR | (uint32_t)QTD_STS_BABBLE | (uint32_t)QTD_STS_XactErr | (uint32_t)QTD_STS_MISS_MF)) != 0U)
        {
            USB_error("qTD error token=0x%x!  0x%x\n", qtd->Token, qtd->Bptr[0]);

            if (qtd->utr->status == 0)
            {
                qtd->utr->status = USBH_ERR_TRANSACTION;
            }
        }
        else
        {
            if ((qtd->Token & (uint32_t)QTD_PID_Msk) != (uint32_t)QTD_PID_SETUP)
            {
                uint32_t xferred_len;

                xferred_len = qtd->xfer_len - (uint32_t)QTD_TODO_LEN(qtd->Token);
                qtd->utr->xfer_len += xferred_len;
                // USB_debug("0x%x  utr->xfer_len += %d\n", qtd->Token, qtd->xfer_len - QTD_TODO_LEN(qtd->Token));
            }
        }

        return 1;
    }

    return 0;
}

static void scan_asynchronous_list(void)
{
    QH_T    *qh;
    qTD_T   *q_pre;
    qTD_T   *qtd;
    qTD_T   *qtd_tmp;

    qh =  QH_PTR(_H_qh->HLink);
    q_pre = USBNULL;

    while (qh != _H_qh)
    {
        // USB_debug("Scan qh=0x%x, 0x%x\n", (int)qh, qh->OL_Token);
        UTR_T   *utr;
        utr = USBNULL;
        qtd = qh->qtd_list;

        while (qtd != USBNULL)
        {
            if (visit_qtd(qtd))                  /* if TRUE, reclaim this qtd             */
            {
                /* qTD is completed, will remove it      */
                utr = qtd->utr;

                if (qtd == qh->qtd_list)
                {
                    qh->qtd_list = qtd->next;    /* unlink the qTD from qtd_list          */
                }
                else
                {
                    q_pre->next = qtd->next;     /* unlink the qTD from qtd_list          */
                }

                qtd_tmp = qtd;                   /* remember this qTD for freeing later   */
                qtd = qtd->next;                 /* advance to the next qTD               */

                qtd_tmp->next = qh->done_list;   /* push this qTD to QH's done list       */
                qh->done_list = qtd_tmp;
            }
            else
            {
                q_pre = qtd;                     /* remember this qTD as a preceder       */
                qtd = qtd->next;                 /* advance to next qTD                   */
            }
        }

        QH_T const *qh_tmp;
        qh_tmp = qh;
        qh = QH_PTR(qh->HLink);                  /* advance to the next QH                */

        /* If all TDs are done, call-back to requester and then remove this QH.           */
        if ((qh_tmp->qtd_list == USBNULL) && (utr != USBNULL))
        {
            // printf("T %d [%d]\n", (qh_tmp->Chrst>>8)&0xf, (qh_tmp->OL_Token&QTD_DT) ? 1 : 0);
            if ((qh_tmp->OL_Token & QTD_DT) != 0U)
            {
                utr->ep->bToggle = 1;
            }
            else
            {
                utr->ep->bToggle = 0;
            }

            utr->bIsTransferDone = 1;

            if (utr->func != USBNULL)
            {
                utr->func(utr);
            }

            _ehci->UCMDR |= HSUSBH_UCMDR_IAAD_Msk;   /* trigger IAA to reclaim done_list  */
        }
    }
}

static void scan_periodic_frame_list(void)
{
    QH_T    *qh;
    qTD_T   *qtd;
    qTD_T   *qNext;
    UTR_T   *utr;

    /*------------------------------------------------------------------------------------*/
    /* Scan interrupt frame list                                                          */
    /*------------------------------------------------------------------------------------*/
    qh =  _Iqh[NUM_IQH - 1U];

    while (qh != USBNULL)
    {
        qtd = qh->qtd_list;

        if (qtd == USBNULL)
        {
            /* empty QH                                   */
            qh = QH_PTR(qh->HLink);         /* advance to the next QH                     */
            continue;
        }

        while (qtd != USBNULL)
        {
            qNext = qtd->next;

            if (visit_qtd(qtd))                 /* if TRUE, reclaim this qtd                  */
            {
                qh->qtd_list = qtd->next;       /* proceed to next qTD or USBNULL                */
                qtd->next = qh->done_list;      /* push qTD into the done list                */
                qh->done_list = qtd;            /* move qTD to done list                      */
            }

            qtd = qNext;
        }

        qtd = qh->done_list;

        /* If all TDs are done, call-back to requester and then remove this QH.           */
        while (qtd != USBNULL)
        {
            if (qh->qtd_list == USBNULL)
            {
                utr = qtd->utr;

                if (qh->OL_Token & QTD_DT)
                {
                    utr->ep->bToggle = 1;
                }
                else
                {
                    utr->ep->bToggle = 0;
                }

                utr->bIsTransferDone = 1;

                if (utr->func != USBNULL)
                {
                    utr->func(utr);
                }

                _ehci->UCMDR |= HSUSBH_UCMDR_IAAD_Msk;   /* trigger IAA to reclaim done_list  */
            }

            qtd = qtd->next;
        }

        qh = QH_PTR(qh->HLink);                  /* advance to the next QH                */
    }

    /*------------------------------------------------------------------------------------*/
    /* Scan isochronous frame list                                                          */
    /*------------------------------------------------------------------------------------*/

    scan_isochronous_list();
}

static void iaad_remove_qh(void)
{
    QH_T    *qh;
    qTD_T   *qtd;

    /*------------------------------------------------------------------------------------*/
    /* Remove all QHs in qh_remove_list...                                                */
    /*------------------------------------------------------------------------------------*/
    while (qh_remove_list != USBNULL)
    {
        qh = qh_remove_list;
        qh_remove_list = qh->next;

        // USB_debug("iaad_remove_qh - remove QH 0x%x\n", (int)qh);

        while (qh->done_list != USBNULL)    /* we can free the qTDs now                   */
        {
            qtd = qh->done_list;
            qh->done_list = qtd->next;
            free_ehci_qTD(qtd);
        }

        if (qh->qtd_list != USBNULL)           /* still have incomplete qTDs?               */
        {
            UTR_T   *utr;
            utr = qh->qtd_list->utr;

            while (qh->qtd_list != USBNULL)
            {
                qtd = qh->qtd_list;
                qh->qtd_list = qtd->next;
                free_ehci_qTD(qtd);
            }

            utr->status = USBH_ERR_ABORT;
            utr->bIsTransferDone = 1;

            if (utr->func != USBNULL)
            {
                utr->func(utr);             /* call back                                  */
            }
        }

        if (qh->dummy != USBNULL)
        {
            free_ehci_qTD(qh->dummy);
        }

        free_ehci_QH(qh);                   /* free the QH                                */
    }

    /*------------------------------------------------------------------------------------*/
    /* Free all qTD in done_list of each asynchronous QH                                  */
    /*------------------------------------------------------------------------------------*/
    qh =  QH_PTR(_H_qh->HLink);

    while (qh != _H_qh)
    {
        while (qh->done_list != USBNULL)    /* we can free the qTDs now                   */
        {
            qtd = qh->done_list;
            qh->done_list = qtd->next;
            free_ehci_qTD(qtd);
        }

        qh = QH_PTR(qh->HLink);                  /* advance to the next QH                */
    }

    /*------------------------------------------------------------------------------------*/
    /* Free all qTD in done_list of each QH of periodic frame list                        */
    /*------------------------------------------------------------------------------------*/
    qh =  _Iqh[NUM_IQH - 1U];

    while (qh != USBNULL)
    {
        while (qh->done_list != USBNULL)    /* we can free the qTDs now                   */
        {
            qtd = qh->done_list;

            if (qtd == qtd->next)
            {

                free_ehci_qTD(qtd);
                qh->done_list = USBNULL;
                break;
            }

            qh->done_list = qtd->next;
            free_ehci_qTD(qtd);
        }

        qh = QH_PTR(qh->HLink);                  /* advance to the next QH                */
    }
}

static void sync_remove_qtd_done(void)
{
    QH_T    *qh;
    qTD_T   *qtd;

    qh =  _Iqh[NUM_IQH - 1U];

    while (qh != USBNULL)
    {
        while (qh->done_list != USBNULL)    /* we can free the qTDs now                   */
        {
            qtd = qh->done_list;

            if (qtd == qtd->next)
            {
                free_ehci_qTD(qtd);
                qh->done_list = USBNULL;
                break;
            }

            qh->done_list = qtd->next;
            free_ehci_qTD(qtd);
        }

        qh = QH_PTR(qh->HLink);                  /* advance to the next QH                */
    }
}

//static irqreturn_t ehci_irq (struct usb_hcd *hcd)
NVT_ITCM void EHCI_IRQHandler(void)
{
    uint32_t  intsts;

    intsts = _ehci->USTSR;
    _ehci->USTSR = intsts;                  /* clear interrupt status                     */

    //USB_debug("Eirq USTSR=0x%x\n", intsts);

    if ((intsts & HSUSBH_USTSR_UERRINT_Msk) != 0U)
    {
        //TBD...
    }

    if ((intsts & HSUSBH_USTSR_USBINT_Msk) != 0U)
    {
        /* some transfers completed, travel asynchronous */
        /* and periodic lists to find and reclaim them.  */
        scan_asynchronous_list();

        sync_remove_qtd_done();

        scan_periodic_frame_list();
    }

    if ((intsts & HSUSBH_USTSR_IAA_Msk) != 0U)
    {
        iaad_remove_qh();
    }
}

static UDEV_T *ehci_find_device_by_port(int port)
{
    UDEV_T  *udev;

    udev = g_udev_list;

    while (udev != USBNULL)
    {
        if ((udev->parent == USBNULL) && (udev->port_num == (uint8_t)port) && (udev->speed == SPEED_HIGH))
        {
            return udev;
        }

        udev = udev->next;
    }

    return USBNULL;
}

static int ehci_rh_port_reset(int port)
{
    int32_t  retry;
    uint32_t  reset_time;
    int32_t  port_ready;

    reset_time = (uint32_t)PORT_RESET_TIME_MS;
    port_ready = 0;

    for (retry = 0; retry < PORT_RESET_RETRY; retry++)
    {
        uint32_t  t0;
        uint32_t  reset_delay_ticks;

        _ehci->UPSCR[port] = (_ehci->UPSCR[port] | HSUSBH_UPSCR_PRST_Msk) & ~HSUSBH_UPSCR_PE_Msk;
        reset_delay_ticks = reset_time / 10U;

        t0 = get_ticks();

        while ((get_ticks() - t0) < (reset_delay_ticks + 1U))
        {
            ;  /* wait at least 50 ms        */
        }

        _ehci->UPSCR[port] &= ~HSUSBH_UPSCR_PRST_Msk;

        t0 = get_ticks();

        while ((get_ticks() - t0) < (reset_delay_ticks + 1U))
        {
            if (((_ehci->UPSCR[port] & HSUSBH_UPSCR_CCS_Msk) == 0U) ||
                    ((_ehci->UPSCR[port] & (HSUSBH_UPSCR_CCS_Msk | HSUSBH_UPSCR_PE_Msk)) == (HSUSBH_UPSCR_CCS_Msk | HSUSBH_UPSCR_PE_Msk)))
            {
                port_ready = 1;
                break;
            }
        }

        if (port_ready != 0)
        {
            break;
        }

        reset_time += (uint32_t)PORT_RESET_RETRY_INC_MS;
    }

    if (port_ready == 0)
    {
        USB_debug("EHCI port %d - port reset failed!\n", port + 1);
        return USBH_ERR_PORT_RESET;
    }

    if ((_ehci->UPSCR[port] & HSUSBH_UPSCR_CCS_Msk) == 0U)    /* check again if device disconnected */
    {
        USB_debug("EHCI port 0x%x - port Disconnected!\n", _ehci->UPSCR[port]);
        _ehci->UPSCR[port] |= HSUSBH_UPSCR_CSC_Msk;          /* clear CSC                          */
        return USBH_ERR_DISCONNECTED;
    }

    _ehci->UPSCR[port] |= HSUSBH_UPSCR_PEC_Msk;              /* clear port enable change status    */
    return USBH_OK;                                          /* port reset success                 */
}

static int ehci_rh_polling(void)
{
    UDEV_T    *udev;
    uint32_t  connect_status;
    uint32_t  t0;

    if ((_ehci->UPSCR[0] & HSUSBH_UPSCR_CSC_Msk) == 0U)
    {
        return 0;
    }

    /*------------------------------------------------------------------------------------*/
    /*  connect status change                                                             */
    /*------------------------------------------------------------------------------------*/

    USB_debug("EHCI port1 status change: 0x%x\n", _ehci->UPSCR[0]);

    /*--------------------------------------------------------------------------------*/
    /*  Disconnect the devices attached to this port.                                 */
    /*--------------------------------------------------------------------------------*/
    while (1)
    {
        udev = ehci_find_device_by_port(1);

        if (udev == USBNULL)
        {
            break;
        }

        disconnect_device(udev);
    }

    /*--------------------------------------------------------------------------------*/
    /*  Port de-bounce                                                                */
    /*--------------------------------------------------------------------------------*/
    t0 = get_ticks();
    connect_status = _ehci->UPSCR[0] & HSUSBH_UPSCR_CCS_Msk;

    while ((get_ticks() - t0) < ((uint32_t)HUB_DEBOUNCE_TIME / 10U))
    {
        if (connect_status != (_ehci->UPSCR[0] & HSUSBH_UPSCR_CCS_Msk))
        {
            /* reset stable time counting                                             */
            t0 = get_ticks();
            connect_status = _ehci->UPSCR[0] & HSUSBH_UPSCR_CCS_Msk;
        }
    }

    _ehci->UPSCR[0] |= HSUSBH_UPSCR_CSC_Msk;     /* clear connect status change bit   */

    if (connect_status == HSUSBH_UPSCR_CCS_Msk)
    {
        /*--------------------------------------------------------------------------------*/
        /*  A new device connected.                                                       */
        /*--------------------------------------------------------------------------------*/
        if (ehci_rh_port_reset(0) != USBH_OK)
        {
            USB_debug("Reset Port fail\r\n");
            /* port reset failed, maybe an USB 1.1 device */
            _ehci->UPSCR[0] |= HSUSBH_UPSCR_PO_Msk;     /* change port owner to OHCI      */
            _ehci->UPSCR[0] |= HSUSBH_UPSCR_CSC_Msk;    /* clear all status change bits   */
            return 0;
        }

        /*
         *  Port reset success. Start to enumerate this new device.
         */
        udev = alloc_device();

        if (udev == USBNULL)
        {
            return 0;                       /* out-of-memory, do nothing...               */
        }

        udev->parent = USBNULL;
        udev->port_num = 1;
        udev->speed = SPEED_HIGH;
        udev->hc_driver = &ehci_driver;

        int ret;
        ret = connect_device(udev);

        if (ret < 0)
        {
            USB_error("connect_device error! [%d]\n", ret);
            free_device(udev);
        }
    }
    else
    {
        /*
         *  Device disconnected
         */
        while (1)
        {
            udev = ehci_find_device_by_port(1);

            if (udev == USBNULL)
            {
                break;
            }

            disconnect_device(udev);
        }
    }

    return 1;
}

HC_DRV_T  ehci_driver =
{
    ehci_init,               /* init               */
    ehci_shutdown,           /* shutdown           */
    ehci_suspend,            /* suspend            */
    ehci_resume,             /* resume             */
    ehci_ctrl_xfer,          /* ctrl_xfer          */
    ehci_bulk_xfer,          /* bulk_xfer          */
    ehci_int_xfer,           /* int_xfer           */
    ehci_iso_xfer,           /* iso_xfer           */
    ehci_quit_xfer,          /* quit_xfer          */
    ehci_rh_port_reset,      /* rthub_port_reset   */
    ehci_rh_polling          /* rthub_polling      */
};
#endif
/// @endcond HIDDEN_SYMBOLS
