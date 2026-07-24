/**************************************************************************//**
 * @file     ehci_iso.c
 * @version  V1.00
 * @brief    USB EHCI isochronous transfer driver.
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
ISO_EP_T  *iso_ep_list;                     /* list of activated isochronous pipes        */

static int ehci_iso_split_xfer(UTR_T *utr, ISO_EP_T *iso_ep);

/*
 *  Inspect the iTD can be reclaimed or not. If yes, collect the transaction results.
 *  Return:  1 - reclaimed
 *           0 - not completed
 */
static int  review_itd(iTD_T *itd)
{
    UTR_T      *utr;
    uint32_t   frnidx = itd->sched_frnidx;
    uint32_t   now_frame = (_ehci->UFINDR >> 3) & 0x3FFU;
    int        i;
    int        fidx;

    // printf("R - %d %d, 0x%x\n", now_frame, frnidx, itd->Transaction[0]);

    if (now_frame == frnidx)
    {
        for (i = 0; i < 8; i++)
        {
            if ((itd->Transaction[i] & ITD_STATUS_ACTIVE) != 0U)
            {
                return 0;                   /* have any not completed frames              */
            }
        }
    }
    else if (now_frame > frnidx)
    {
        if ((now_frame - frnidx) > (uint32_t)EHCI_ISO_RCLM_RANGE)
        {
            return 0;                       /* don't touch it                             */
        }
    }
    else
    {
        if (((now_frame + FL_SIZE) - frnidx) > (uint32_t)EHCI_ISO_RCLM_RANGE)
        {
            return 0;                       /* don't touch it                             */
        }
    }

    /*
     *  Reclaim this iTD
     */
    utr = itd->utr;
    fidx = itd->fidx;

    for (i = 0; i < 8; i++)
    {
        if ((itd->trans_mask & (0x1U << (uint32_t)i)) == 0U)
        {
            continue;                       /* not scheduled micro-frame                  */
        }

        if (ITD_STATUS(itd->Transaction[i]) != 0U)
        {
            if ((itd->Transaction[i] & ITD_STATUS_ACTIVE) != 0U)
            {
                utr->iso_status[fidx] = USBH_ERR_NOT_ACCESS0;
                utr->status = USBH_ERR_NOT_ACCESS0;
            }
            else if ((itd->Transaction[i] & ITD_STATUS_BABBLE) != 0U)
            {
                utr->iso_status[fidx] = USBH_ERR_BABBLE_DETECTED;
                utr->status = USBH_ERR_TRANSFER;
            }
            else if ((itd->Transaction[i] & ITD_STATUS_BUFF_ERR) != 0U)
            {
                utr->iso_status[fidx] = USBH_ERR_DATA_BUFF;
                utr->status = USBH_ERR_TRANSFER;
            }
            else
            {
                utr->iso_status[fidx] = USBH_ERR_TRANSACTION;
                utr->status = USBH_ERR_TRANSFER;
            }
        }
        else
        {
            utr->iso_status[fidx] = 0;
            utr->iso_xlen[fidx] = (uint16_t)ITD_XFER_LEN(itd->Transaction[i]);
        }

        fidx++;
    }

    utr->td_cnt--;

    if (utr->td_cnt == 0)                   /* All iTD of this UTR done                   */
    {
        utr->bIsTransferDone = 1;

        if (utr->func != USBNULL)
        {
            utr->func(utr);
        }
    }

    return 1;                               /* to be reclaimed                            */
}

/*
 *  Inspect the siTD can be reclaimed or not. If yes, collect the transaction results.
 *  Return:  1 - reclaimed
 *           0 - not completed
 */
static int  review_sitd(siTD_T *sitd)
{
    UTR_T      *utr;
    uint32_t   frnidx = sitd->sched_frnidx;
    uint32_t   now_frame = (_ehci->UFINDR >> 3) & 0x3FFU;
    int        fidx;

    if (now_frame == frnidx)
    {
        if (SITD_STATUS(sitd->StsCtrl) == SITD_STATUS_ACTIVE)
        {
            return 0;
        }
    }
    else if (now_frame > frnidx)
    {
        if ((now_frame - frnidx) > (uint32_t)EHCI_ISO_RCLM_RANGE)
        {
            return 0;                       /* don't touch it                             */
        }
    }
    else
    {
        if (((now_frame + FL_SIZE) - frnidx) > (uint32_t)EHCI_ISO_RCLM_RANGE)
        {
            return 0;                       /* don't touch it                             */
        }
    }

    /*
     *  Reclaim this siTD
     */
    utr = sitd->utr;
    fidx = sitd->fidx;

    if (SITD_STATUS(sitd->StsCtrl) != 0U)
    {
        if ((sitd->StsCtrl & SITD_STATUS_ACTIVE) != 0U)
        {
            utr->iso_status[fidx] = USBH_ERR_NOT_ACCESS0;
        }
        else if ((sitd->StsCtrl & SITD_BABBLE_DETECTED) != 0U)
        {
            utr->iso_status[fidx] = USBH_ERR_BABBLE_DETECTED;
            utr->status = USBH_ERR_TRANSFER;
        }
        else if ((sitd->StsCtrl & SITD_STATUS_BUFF_ERR) != 0U)
        {
            utr->iso_status[fidx] = USBH_ERR_DATA_BUFF;
            utr->status = USBH_ERR_TRANSFER;
        }
        else
        {
            utr->iso_status[fidx] = USBH_ERR_TRANSACTION;
            utr->status = USBH_ERR_TRANSFER;
        }
    }
    else
    {
        uint32_t TotalBytesToTransfer = (sitd->StsCtrl & SITD_XFER_CNT_Msk) >> SITD_XFER_CNT_Pos;
        utr->iso_xlen[fidx] = (uint16_t)((uint32_t)utr->iso_xlen[fidx] - TotalBytesToTransfer);
        utr->iso_status[fidx] = 0;
    }

    utr->td_cnt--;

    if (utr->td_cnt == 0)                   /* All iTD of this UTR done                   */
    {
        utr->bIsTransferDone = 1;

        if (utr->func != USBNULL)
        {
            utr->func(utr);
        }
    }

    return 1;                               /* to be reclaimed                            */
}

/*
 *  Some iTD/siTD may be scheduled but not serviced due to time missed.
 *  This function scan several earlier frames and drop unserviced iTD/siTD if found.
 */
void scan_isochronous_list(void)
{
    ISO_EP_T   *iso_ep = iso_ep_list;
    iTD_T      *itd;
    iTD_T      *p;
    siTD_T     *sitd;
    siTD_T     *sp;
    uint32_t   frnidx;

    DISABLE_EHCI_IRQ();

    while (iso_ep != USBNULL)                  /* Search all activated iso endpoints         */
    {
        /*--------------------------------------------------------------------------------*/
        /*  Scan all iTDs                                                                 */
        /*--------------------------------------------------------------------------------*/
        itd = iso_ep->itd_list;             /* get the first iTD from iso_ep's iTD list   */
        iTD_T      *itd_pre = USBNULL;

        while (itd != USBNULL)                 /* traverse all iTDs of itd list              */
        {
            if (review_itd(itd))            /* inspect and reclaim iTD                    */
            {
                /*------------------------------------------------------------------------*/
                /*  Remove this iTD from period frame list                                */
                /*------------------------------------------------------------------------*/
                frnidx = itd->sched_frnidx;

                if (_PFList[frnidx] == ITD_HLNK_ITD(itd))
                {
                    /* is the first entry, just change to next     */
                    _PFList[frnidx] = itd->Next_Link;
                }
                else
                {
                    p = ITD_PTR(_PFList[frnidx]);     /* find the preceding iTD            */

                    while ((p != USBNULL) && (ITD_PTR(p->Next_Link) != itd))
                    {
                        p = ITD_PTR(p->Next_Link);
                    }

                    if (p == USBNULL)                    /* link list out of control!         */
                    {
                        USB_error("An iTD lost refernece to periodic frame list! 0x%x -> %d\n", (int)itd, frnidx);
                    }
                    else                              /* remove iTD from list              */
                    {
                        p->Next_Link = itd->Next_Link;
                    }
                }

                /*------------------------------------------------------------------------*/
                /*  Remove this iTD from iso_ep's iTD list                                */
                /*------------------------------------------------------------------------*/
                if (itd_pre == USBNULL)
                {
                    iso_ep->itd_list = itd->next;
                }
                else
                {
                    itd_pre->next = itd->next;
                }

                p = itd->next;
                free_ehci_iTD(itd);
                itd = p;
            }
            else
            {
                itd_pre = itd;
                itd = itd->next;            /* traverse to the next iTD of iTD list       */
            }
        }

        /*--------------------------------------------------------------------------------*/
        /*  Scan all siTDs                                                                */
        /*--------------------------------------------------------------------------------*/
        sitd = iso_ep->sitd_list;           /* get the first siTD from iso_ep's siTD list */
        siTD_T     *sitd_pre = USBNULL;

        while (sitd != USBNULL)                /* traverse all siTDs of sitd list            */
        {
            if (review_sitd(sitd))          /* inspect and reclaim siTD                   */
            {
                /*------------------------------------------------------------------------*/
                /*  Remove this siTD from period frame list                               */
                /*------------------------------------------------------------------------*/
                frnidx = sitd->sched_frnidx;

                if (_PFList[frnidx] == SITD_HLNK_SITD(sitd))
                {
                    /* is the first entry, just change to next     */
                    _PFList[frnidx] = sitd->Next_Link;
                }
                else
                {
                    sp = SITD_PTR(_PFList[frnidx]);   /* find the preceding siTD           */

                    while ((sp != USBNULL) && (SITD_PTR(sp->Next_Link) != sitd))
                    {
                        sp = SITD_PTR(sp->Next_Link);
                    }

                    if (sp == USBNULL)                   /* link list out of control!         */
                    {
                        USB_error("An siTD lost reference to periodic frame list! 0x%x -> %d\n", (int)sitd, frnidx);
                    }
                    else                              /* remove iTD from list              */
                    {
                        sp->Next_Link = sitd->Next_Link;
                    }
                }

                /*------------------------------------------------------------------------*/
                /*  Remove this siTD from iso_ep's siTD list                              */
                /*------------------------------------------------------------------------*/
                if (sitd_pre == USBNULL)
                {
                    iso_ep->sitd_list = sitd->next;
                }
                else
                {
                    sitd_pre->next = sitd->next;
                }

                sp = sitd->next;
                free_ehci_siTD(sitd);
                sitd = sp;
            }
            else
            {
                sitd_pre = sitd;
                sitd = sitd->next;           /* traverse to the next siTD of siTD list     */
            }
        }

        iso_ep = iso_ep->next;
    }

    ENABLE_EHCI_IRQ();
}

static void  write_itd_info(UTR_T const *utr, iTD_T *itd)
{
    UDEV_T     const *udev = utr->udev;
    EP_INFO_T  const *ep = utr->ep;               /* reference to isochronous endpoint          */
    uint32_t   buff_page_addr;
    int        i;
    uint32_t mult;

    mult = ((uint32_t)ep->wMaxPacketSize + 1023U) / 1024U;

    if (mult == 0U)
    {
        mult = 1U;
    }

    buff_page_addr = itd->buff_base & 0xFFFFF000U;     /* 4K page                          */

    for (i = 0; i < 7; i++)
    {
        itd->Bptr[i] = buff_page_addr + (0x1000U * (uint32_t)i);
    }

    /* EndPtr  R  Device Address        */
    itd->Bptr[0] |= (uint32_t)udev->dev_num | (((uint32_t)ep->bEndpointAddress & 0xFU) << ITD_EP_NUM_Pos);
    itd->Bptr[1] |= (uint32_t)ep->wMaxPacketSize / mult;   /* Maximum Packet Size              */

    if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN) /* I/O               */
    {
        itd->Bptr[1] |= ITD_DIR_IN;
    }
    else
    {
        itd->Bptr[1] |= ITD_DIR_OUT;
    }

    itd->Bptr[2] |= mult;                                   /* Mult              */

    /* cppcheck-suppress misra-c2012-17.3 ; DEVIATION: __DSB() is a CMSIS compiler intrinsic; prototype not visible to cppcheck */
    __DSB();

}

static void  write_itd_micro_frame(UTR_T *utr, int fidx, iTD_T *itd, int mf)
{
    uint32_t   buff_addr;


    buff_addr = (uint32_t)(utr->iso_buff[fidx]); /* xfer buffer start address of this frame */

    itd->Transaction[mf] = ITD_STATUS_ACTIVE |                                            /* Status */
                           (((uint32_t)utr->iso_xlen[fidx] & 0xFFFU) << ITD_XLEN_Pos) |   /* Transaction Length */
                           ((buff_addr & 0xFFFFF000U) - (itd->buff_base & 0xFFFFF000U)) | /* PG */
                           (buff_addr & 0xFFFU);                                          /* Transaction offset */

    /* cppcheck-suppress misra-c2012-17.3 ; DEVIATION: __DSB() is a CMSIS compiler intrinsic; prototype not visible to cppcheck */
    __DSB();
}

static void remove_iso_ep_from_list(ISO_EP_T *iso_ep)
{
    ISO_EP_T  *p;

    if (iso_ep_list == iso_ep)
    {
        iso_ep_list = iso_ep->next;         /* it's the first entry, remove it            */
        return;
    }

    p = iso_ep_list;                        /* find the previous entry of iso_ep          */

    while (p->next != USBNULL)
    {
        if (p->next == iso_ep)
        {
            break;
        }

        p = p->next;
    }

    if (p->next == USBNULL)
    {
        return;                             /* not found                                  */
    }

    p->next = iso_ep->next;                 /* remove iso_ep from list                    */
}

static __inline void  add_itd_to_iso_ep(ISO_EP_T *iso_ep, iTD_T *itd)
{
    iTD_T   *p;

    itd->next = USBNULL;

    if (iso_ep->itd_list == USBNULL)
    {
        iso_ep->itd_list = itd;
        return;
    }

    /*
     * Find the tail entry of iso_ep->itd_list
     */
    p = iso_ep->itd_list;

    while (p->next != USBNULL)
    {
        p = p->next;
    }

    p->next = itd;
}

static void  free_all_itd(iTD_T *itd_list)
{
    iTD_T  *itd;
    iTD_T  *itd_cur = itd_list;

    while (itd_cur != USBNULL)
    {
        itd = itd_cur;
        itd_cur = itd->next;
        free_ehci_iTD(itd);
    }
}

int ehci_iso_xfer(UTR_T *utr)
{
    EP_INFO_T  *ep = utr->ep;               /* reference to isochronous endpoint          */
    ISO_EP_T   *iso_ep;                     /* software iso endpoint descriptor           */
    iTD_T      *itd;
    iTD_T      *itd_next;
    iTD_T      *itd_list = USBNULL;
    int        i;
    int        itd_cnt;
    int        trans_mask;                  /* bit mask of used xfer in an iTD            */
    unsigned int fidx;                      /* index to the 8 iso frames of UTR           */
    int        interval;                    /* frame interval of iTD                      */

    if (ep->hw_pipe != USBNULL)
    {
        iso_ep = (ISO_EP_T *)ep->hw_pipe;   /* get reference of the isochronous endpoint  */

        if (utr->bIsoNewSched != 0U)
        {
            iso_ep->next_frame = (((_ehci->UFINDR + ((uint32_t)EHCI_ISO_DELAY * 8U)) & HSUSBH_UFINDR_FI_Msk) >> 3) & 0x3FFU;
        }
    }
    else
    {
        /* first time transfer of this iso endpoint   */
        iso_ep = usbh_alloc_mem(sizeof(*iso_ep));

        if (iso_ep == USBNULL)
        {
            return USBH_ERR_MEMORY_OUT;
        }

        (void)memset(iso_ep, 0, sizeof(*iso_ep));
        iso_ep->ep = ep;
        iso_ep->next_frame = (((_ehci->UFINDR + ((uint32_t)EHCI_ISO_DELAY * 8U)) & HSUSBH_UFINDR_FI_Msk) >> 3) & 0x3FFU;

        ep->hw_pipe = iso_ep;

        /*
         *  Add this iso_ep into iso_ep_list
         */
        DISABLE_EHCI_IRQ();
        iso_ep->next = iso_ep_list;
        iso_ep_list = iso_ep;
        ENABLE_EHCI_IRQ();
    }

    if (utr->udev->speed == SPEED_FULL)
    {
        return ehci_iso_split_xfer(utr, iso_ep);
    }

    /*------------------------------------------------------------------------------------*/
    /*  Allocate iTDs                                                                     */
    /*------------------------------------------------------------------------------------*/

    if (ep->bInterval < 2U)                 /* transfer interval is 1 micro-frame         */
    {
        trans_mask = 0xFF;
        itd_cnt = 1;                        /* required 1 iTD for one UTR                 */
        interval = 1;                       /* iTD frame interval of this endpoint        */
    }
    else if (ep->bInterval < 4U)            /* transfer interval is 2 micro-frames        */
    {
        trans_mask = 0x55;
        itd_cnt = 2;                        /* required 2 iTDs for one UTR                */
        interval = 1;                       /* iTD frame interval of this endpoint        */
    }
    else if (ep->bInterval < 8U)            /* transfer interval is 4 micro-frames        */
    {
        trans_mask = 0x44;
        itd_cnt = 4;                        /* required 4 iTDs for one UTR                */
        interval = 1;                       /* iTD frame interval of this endpoint        */
    }
    else if (ep->bInterval < 16U)           /* transfer interval is 8 micro-frames        */
    {
        trans_mask = 0x08;                  /* there's 1 transfer in one iTD              */
        itd_cnt = 8;                        /* required 8 iTDs for one UTR                */
        interval = 1;                       /* iTD frame interval of this endpoint        */
    }
    else if (ep->bInterval < 32U)           /* transfer interval is 16 micro-frames       */
    {
        trans_mask = 0x10;                  /* there's 1 transfer in one iTD              */
        itd_cnt = 8;                        /* required 8 iTDs for one UTR                */
        interval = 2;                       /* iTD frame interval of this endpoint        */
    }
    else if (ep->bInterval < 64U)           /* transfer interval is 32 micro-frames       */
    {
        trans_mask = 0x02;                  /* there's 1 transfer in one iTD              */
        itd_cnt = 8;                        /* required 8 iTDs for one UTR                */
        interval = 4;                       /* iTD frame interval of this endpoint        */
    }
    else                                    /* transfer interval is 64 micro-frames       */
    {
        trans_mask = 0x04;                  /* there's 1 transfer in one iTD              */
        itd_cnt = 8;                        /* required 8 iTDs for one UTR                */
        interval = 8;                       /* iTD frame interval of this endpoint        */
    }

    for (i = 0; i < itd_cnt; i++)           /* allocate all iTDs required by UTR          */
    {
        itd = alloc_ehci_iTD();

        if (itd == USBNULL)
        {
            free_all_itd(itd_list);
            return USBH_ERR_MEMORY_OUT;
        }

        if (itd_list == USBNULL)               /* link all iTDs                              */
        {
            itd_list = itd;
        }
        else
        {
            itd->next = itd_list;
            itd_list = itd;
        }
    }

    utr->td_cnt = itd_cnt;

    /*------------------------------------------------------------------------------------*/
    /*  Fill and link all iTDs                                                            */
    /*------------------------------------------------------------------------------------*/

    utr->iso_sf = iso_ep->next_frame;
    fidx = 0;                               /* index to UTR iso frmes (total IF_PER_UTR)  */

    itd = itd_list;

    while (itd != USBNULL)
    {
        if (fidx >= IF_PER_UTR)             /* unlikely                                   */
        {
            USB_error("EHCI driver ITD bug!?\n");
            free_all_itd(itd_list);
            return USBH_ERR_MEMORY_OUT;
        }

        itd->utr = utr;
        itd->fidx = fidx;                   /* index to UTR's n'th IF_PER_UTR frame       */
        itd->buff_base = (uint32_t)(utr->iso_buff[fidx]);    /* iTD buffer base is buffer of the first UTR iso frame serviced by this iTD */
        itd->trans_mask = trans_mask;

        write_itd_info(utr, itd);

        for (i = 0; i < 8; i++)             /* settle xfer into micro-frames              */
        {
            if (((uint32_t)trans_mask & (0x1U << (uint32_t)i)) == 0U)
            {
                itd->Transaction[i] = 0;    /* not accesed                                */
                continue;                   /* not scheduled micro-frame                  */
            }

            write_itd_micro_frame(utr, fidx, itd, i);

            fidx++;                         /* preceed to next UTR iso frame              */

            if (fidx == IF_PER_UTR)         /* is the last scheduled micro-frame?         */
            {
                /* raise interrupt on completed               */
                itd->Transaction[i] |= ITD_IOC;
                break;
            }
        }

        itd_next = itd->next;               /* remember the next itd                      */

        // USB_debug("Link iTD 0x%x, %d\n", (int)itd, iso_ep->next_frame);
        /*
         *  Link iTD to period frame list
         */
        DISABLE_EHCI_IRQ();
        itd->sched_frnidx = iso_ep->next_frame;       /* remember it for reclamation scan */
        add_itd_to_iso_ep(iso_ep, itd);               /* add to software itd list         */
        itd->Next_Link = _PFList[itd->sched_frnidx];  /* keep the next link               */
        _PFList[itd->sched_frnidx] = ITD_HLNK_ITD(itd);
        iso_ep->next_frame = (iso_ep->next_frame + (uint32_t)interval) % FL_SIZE;
        ENABLE_EHCI_IRQ();

        itd = itd_next;
    }

    _ehci->UCMDR |= HSUSBH_UCMDR_PSEN_Msk;      /* periodic list enable                   */
    return 0;
}

static __inline void  add_sitd_to_iso_ep(ISO_EP_T *iso_ep, siTD_T *sitd)
{
    siTD_T   *p;

    sitd->next = USBNULL;

    if (iso_ep->sitd_list == USBNULL)
    {
        iso_ep->sitd_list = sitd;
        return;
    }

    /*
     * Find the tail entry of iso_ep->itd_list
     */
    p = iso_ep->sitd_list;

    while (p->next != USBNULL)
    {
        p = p->next;
    }

    p->next = sitd;
}

static void  write_sitd_info(UTR_T const *utr, siTD_T *sitd)
{
    static const uint16_t sitd_OUT_Smask [] = { 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f };
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  const *ep = utr->ep;               /* reference to isochronous endpoint          */
    uint32_t   buff_page_addr;
    uint32_t   xlen = utr->iso_xlen[sitd->fidx];
    uint32_t   scnt;

    sitd->Chrst = ((uint32_t)udev->port_num << SITD_PORT_NUM_Pos) |
                  ((uint32_t)udev->parent->iface->udev->dev_num << SITD_HUB_ADDR_Pos) |
                  (((uint32_t)ep->bEndpointAddress & 0xFU) << SITD_EP_NUM_Pos) |
                  ((uint32_t)udev->dev_num << SITD_DEV_ADDR_Pos);

    buff_page_addr = ((uint32_t)utr->iso_buff[sitd->fidx]) & 0xFFFFF000U;
    sitd->Bptr[0] = (uint32_t)(utr->iso_buff[sitd->fidx]);
    sitd->Bptr[1] = buff_page_addr + 0x1000U;

    scnt = (xlen + 187U) / 188U;

    if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN)   /* I/O               */
    {
        sitd->Chrst |= SITD_XFER_IN;
        sitd->Sched = (1UL << ((scnt + 2U) & 0x1FU)) - 1U;
        sitd->Sched = (sitd->Sched << 10) | 0x1U;
        //sitd->Sched <<= 1;
    }
    else
    {
        sitd->Chrst |= SITD_XFER_OUT;
        sitd->Sched = sitd_OUT_Smask[scnt - 1U];

        if (scnt > 1U)
        {
            sitd->Bptr[1] |= (0x1U << 3);       /* Transaction position (TP)  01b: Begin  */
        }

        sitd->Bptr[1] |= scnt;                  /* Transaction count (T-Count)            */
    }



    sitd->StsCtrl = (xlen << SITD_XFER_CNT_Pos) | SITD_STATUS_ACTIVE;

    if ((uint32_t)sitd->fidx == (IF_PER_UTR - 1U))
    {
        sitd->StsCtrl |= SITD_IOC;
    }

    sitd->BackLink = SITD_LIST_END;
}

static void ehci_sitd_adjust_schedule(siTD_T *sitd)
{
    siTD_T     const *hlink = (siTD_T *)_PFList[sitd->sched_frnidx];
    uint32_t   uframe_mask = 0U;

    while ((hlink != USBNULL) && (HLINK_IS_TERMINATED(hlink) == 0) && (HLINK_IS_SITD(hlink) != 0))
    {
        hlink = SITD_PTR(hlink);

        if (hlink != sitd)
        {
            if ((hlink->Chrst & SITD_XFER_IO_Msk) == SITD_XFER_IN)
            {
                uframe_mask |= (hlink->Sched & 0xFFU);         /* mark micro-frames used by IN S-mask   */
                uframe_mask |= ((hlink->Sched >> 8) & 0xFFU);  /* mark micro-frames used by IN C-mask   */
            }
            else
            {
                uframe_mask |= (hlink->Sched & 0xFFU);         /* mark micro-frames used by OUT S-mask  */
            }
        }

        hlink = SITD_PTR(hlink->Next_Link);
    }

    uframe_mask = uframe_mask | (uframe_mask << 8);          /* mark both S-mask and C-mask           */

    if (uframe_mask != 0U)
    {
        /*
         *  Shift afterward one micro-frame until no conflicts.
         */
        while (1)
        {
            if ((sitd->Sched & uframe_mask) != 0U)
            {
                sitd->Sched = (sitd->Sched & 0xFFFF0000U) | ((sitd->Sched << 1) & 0xFFFFU);
            }
            else
            {
                break;                      /* no conflit, done.                          */
            }
        }
    }
}

static void  free_all_sitd(siTD_T *sitd_list)
{
    siTD_T  *sitd;
    siTD_T  *sitd_cur = sitd_list;

    while (sitd_cur != USBNULL)
    {
        sitd = sitd_cur;
        sitd_cur = sitd->next;
        free_ehci_siTD(sitd);
    }
}

static int ehci_iso_split_xfer(UTR_T *utr, ISO_EP_T *iso_ep)
{
    EP_INFO_T  const *ep = utr->ep;               /* reference to isochronous endpoint          */
    siTD_T     *sitd;
    siTD_T     *sitd_next;
    siTD_T     *sitd_list = USBNULL;
    unsigned int i;
    unsigned int fidx;                      /* index to the 8 iso frames of UTR           */

    if (utr->udev->parent == USBNULL)
    {
        USB_error("siso xfer - parent lost!\n");
        return USBH_ERR_INVALID_PARAM;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Allocate siTDs                                                                    */
    /*------------------------------------------------------------------------------------*/
    for (i = 0; i < IF_PER_UTR; i++)        /* allocate all siTDs required by UTR         */
    {
        sitd = alloc_ehci_siTD();

        if (sitd == USBNULL)
        {
            free_all_sitd(sitd_list);
            return USBH_ERR_MEMORY_OUT;
        }

        if (sitd_list == USBNULL)               /* link all siTDs                             */
        {
            sitd_list = sitd;
        }
        else
        {
            sitd->next = sitd_list;
            sitd_list = sitd;
        }
    }

    utr->td_cnt = IF_PER_UTR;

    /*------------------------------------------------------------------------------------*/
    /*  Fill and link all siTDs                                                           */
    /*------------------------------------------------------------------------------------*/

    utr->iso_sf = iso_ep->next_frame;
    fidx = 0;                               /* index to UTR iso frmes (total IF_PER_UTR)  */

    sitd = sitd_list;

    while (sitd != USBNULL)
    {
        if (fidx >= IF_PER_UTR)             /* unlikely                                   */
        {
            USB_error("EHCI driver siTD bug!?\n");
            free_all_sitd(sitd_list);
            return USBH_ERR_MEMORY_OUT;
        }

        sitd->utr = utr;
        sitd->fidx = fidx;                   /* index to UTR's n'th IF_PER_UTR frame       */

        write_sitd_info(utr, sitd);

        sitd_next = sitd->next;              /* remember the next itd                      */

        // USB_debug("Link iTD 0x%x, %d\n", (int)itd, iso_ep->next_frame);
        /*
         *  Link iTD to period frame list
         */
        sitd->sched_frnidx = iso_ep->next_frame;      /* remember it for reclamation scan */
        DISABLE_EHCI_IRQ();
        ehci_sitd_adjust_schedule(sitd);
        add_sitd_to_iso_ep(iso_ep, sitd);             /* add to software itd list         */
        sitd->Next_Link = _PFList[sitd->sched_frnidx];/* keep the next link               */
        _PFList[sitd->sched_frnidx] = SITD_HLNK_SITD(sitd);
        iso_ep->next_frame = (iso_ep->next_frame + (uint32_t)ep->bInterval) % FL_SIZE;
        ENABLE_EHCI_IRQ();

        sitd = sitd_next;
        fidx++;
    }

    _ehci->UCMDR |= HSUSBH_UCMDR_PSEN_Msk;      /* periodic list enable                   */
    return 0;
}

/*
 *  If it's an isochronous endpoint, quit current transfer via UTR or hardware EP.
 */
int ehci_quit_iso_xfer(UTR_T *utr, EP_INFO_T *ep)
{
    ISO_EP_T   *iso_ep;
    iTD_T      *itd;
    iTD_T      *itd_next;
    iTD_T      *p;
    siTD_T     *sitd;
    siTD_T     *sitd_next;
    siTD_T     *sp;
    UTR_T      *utr_cur;
    EP_INFO_T  *ep_cur = ep;
    uint32_t   frnidx;
    uint32_t   now_frame;

    if (ep_cur == USBNULL)
    {
        if (utr == USBNULL)
        {
            return USBH_ERR_NOT_FOUND;
        }

        if (utr->ep == USBNULL)
        {
            return USBH_ERR_NOT_FOUND;
        }

        ep_cur = utr->ep;
    }

    if ((ep_cur->bmAttributes & EP_ATTR_TT_MASK) != EP_ATTR_TT_ISO)
    {
        return USBH_ERR_NOT_FOUND;          /* not isochronous endpoint                   */
    }

    /*------------------------------------------------------------------------------------*/
    /*  It's an iso endpoint. Remove it as required.                                      */
    /*------------------------------------------------------------------------------------*/
    iso_ep = iso_ep_list;

    while (iso_ep != USBNULL)                  /* Search all activated iso endpoints         */
    {
        if (iso_ep->ep == ep_cur)
        {
            break;
        }

        iso_ep = iso_ep->next;
    }

    if (iso_ep == USBNULL)
    {
        return 0;                           /* should have been removed                   */
    }

    itd = iso_ep->itd_list;                 /* get the first iTD from iso_ep's iTD list   */

    while (itd != USBNULL)                     /* traverse all iTDs of itd list              */
    {
        itd_next = itd->next;               /* remember the next iTD                      */
        utr_cur = itd->utr;

        /*--------------------------------------------------------------------------------*/
        /*  Remove this iTD from period frame list                                        */
        /*--------------------------------------------------------------------------------*/
        frnidx = itd->sched_frnidx;

        /*
         *  Prevent to race with Host Controller. If the iTD to be removed is located in
         *  current or next frame, wait until HC passed through it.
         */
        while (1)
        {
            now_frame = (_ehci->UFINDR >> 3) & 0x3FFU;

            if ((now_frame == frnidx) || (((now_frame + 1U) % 1024U) == frnidx))
            {
                continue;
            }

            break;
        }

        if (_PFList[frnidx] == ITD_HLNK_ITD(itd))
        {
            /* is the first entry, just change to next     */
            _PFList[frnidx] = itd->Next_Link;
        }
        else
        {
            p = ITD_PTR(_PFList[frnidx]);   /* find the preceding iTD                     */

            while ((p != USBNULL) && (ITD_PTR(p->Next_Link) != itd))
            {
                p = ITD_PTR(p->Next_Link);
            }

            if (p == USBNULL)                  /* link list out of control!                  */
            {
                USB_error("ehci_quit_iso_xfer - An iTD lost reference to periodic frame list! 0x%x on %d\n", (int)itd, frnidx);
            }
            else                            /* remove iTD from list                       */
            {
                p->Next_Link = itd->Next_Link;
            }
        }

        utr_cur->status = USBH_ERR_ABORT;
        utr_cur->td_cnt--;

        if (utr_cur->td_cnt == 0)           /* All iTD of this UTR done                   */
        {
            utr_cur->bIsTransferDone = 1;

            if (utr_cur->func != USBNULL)
            {
                utr_cur->func(utr_cur);
            }
        }

        free_ehci_iTD(itd);
        itd = itd_next;
    }

    /*------------------------------------------------------------------------------------*/
    /*   Remove this siTD from period frame list                                          */
    /*------------------------------------------------------------------------------------*/

    sitd = iso_ep->sitd_list;

    while (sitd != USBNULL)
    {
        sitd_next = sitd->next;
        utr_cur = sitd->utr;
        frnidx = sitd->sched_frnidx;

        /* Wait until HC passed through it */
        while (1)
        {
            now_frame = (_ehci->UFINDR >> 3) & 0x3FFU;

            if ((now_frame == frnidx) || (((now_frame + 1U) % 1024U) == frnidx))
            {
                continue;
            }

            break;
        }

        /* Remove from Periodic Frame List */
        if (_PFList[frnidx] == SITD_HLNK_SITD(sitd))
        {
            _PFList[frnidx] = sitd->Next_Link;
        }
        else
        {
            sp = SITD_PTR(_PFList[frnidx]);

            while (sp != USBNULL)
            {
                if (SITD_PTR(sp->Next_Link) == sitd)
                {
                    break;
                }

                sp = SITD_PTR(sp->Next_Link);
            }

            if (sp == USBNULL)
            {
                USB_error("ehci_quit_iso_xfer - An siTD lost reference! 0x%x\n", (int)sitd);
            }
            else
            {
                sp->Next_Link = sitd->Next_Link;
            }
        }

        utr_cur->status = USBH_ERR_ABORT;
        utr_cur->td_cnt--;

        if (utr_cur->td_cnt == 0)
        {
            utr_cur->bIsTransferDone = 1;

            if (utr_cur->func != USBNULL)
            {
                utr_cur->func(utr_cur);
            }
        }

        free_ehci_siTD(sitd);
        sitd = sitd_next;
    }

    /*
     *  Remove iso_ep from iso_ep_list
     */
    remove_iso_ep_from_list(iso_ep);
    (void)usbh_free_mem(iso_ep, sizeof(*iso_ep));/* free this iso_ep                      */
    ep_cur->hw_pipe = USBNULL;

    if (iso_ep_list == USBNULL)
    {
        _ehci->UCMDR &= ~HSUSBH_UCMDR_PSEN_Msk;
    }

    return 0;
}
#endif
/// @endcond HIDDEN_SYMBOLS
