/**************************************************************************//**
 * @file     mem_alloc.c
 * @version  V1.00
 * @brief    USB Host library memory allocation functions.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"

/// @cond HIDDEN_SYMBOLS

//#define MEM_DEBUG

#ifdef MEM_DEBUG
    #define mem_debug       printf
#else
    #define mem_debug(...)
#endif


#ifdef __ICCARM__
    #if (NVT_DCACHE_ON == 1)
        /* QH/QTD/iTD/siTD are placed in a non-cacheable region */
        #pragma data_alignment = 32
        NVT_NONCACHEABLE static uint8_t _hw_mem_pool[HW_MEM_UNIT_NUM][HW_MEM_UNIT_SIZE];                                 /* Periodic frame list        */
    #else
        #pragma data_alignment = 32
        static uint8_t _hw_mem_pool[HW_MEM_UNIT_NUM][HW_MEM_UNIT_SIZE];
    #endif
#else
    #if (NVT_DCACHE_ON == 1)
        /* QH/QTD/iTD/siTD are placed in a non-cacheable region */
        NVT_NONCACHEABLE static uint8_t _hw_mem_pool[HW_MEM_UNIT_NUM][HW_MEM_UNIT_SIZE] __attribute__((aligned(4096)));  /* Periodic frame list        */
    #else
        static uint8_t _hw_mem_pool[HW_MEM_UNIT_NUM][HW_MEM_UNIT_SIZE] __attribute__((aligned(4096)));  /* Periodic frame list        */
    #endif
#endif

#ifdef __ICCARM__
    #if (NVT_DCACHE_ON == 1)
        /* Device descriptor / UTR and data buffer are placed in a non-cacheable region */
        #pragma data_alignment = 32
        NVT_NONCACHEABLE static uint8_t _dma_mem_pool[DMA_MEM_UNIT_NUM][DMA_MEM_UNIT_SIZE];                                 /* Periodic frame list        */
    #else
        #pragma data_alignment = 32
        static uint8_t _dma_mem_pool[DMA_MEM_UNIT_NUM][DMA_MEM_UNIT_SIZE];
    #endif
#else
    #if (NVT_DCACHE_ON == 1)
        /* Device descriptor / UTR and data buffer are placed in a non-cacheable region */
        NVT_NONCACHEABLE static uint8_t _dma_mem_pool[DMA_MEM_UNIT_NUM][DMA_MEM_UNIT_SIZE] __attribute__((aligned(4096)));  /* Periodic frame list        */
    #else
        static uint8_t _dma_mem_pool[DMA_MEM_UNIT_NUM][DMA_MEM_UNIT_SIZE] __attribute__((aligned(4096)));  /* Periodic frame list        */
    #endif
#endif

#if (NVT_DCACHE_ON == 1)
    /* hw/dma using memory are placed in a non-cacheable region */
    NVT_NONCACHEABLE static uint8_t  _hw_unit_used[HW_MEM_UNIT_NUM];
    NVT_NONCACHEABLE static uint8_t  _dma_unit_used[DMA_MEM_UNIT_NUM];
#else
    static uint8_t  _hw_unit_used[HW_MEM_UNIT_NUM];
    static uint8_t  _dma_unit_used[DMA_MEM_UNIT_NUM];
#endif

static volatile int  _usbh_hw_mem_used;
static volatile int  _usbh_hw_max_mem_used;
static volatile int  _hw_mem_pool_used;

static volatile int  _usbh_dma_mem_used;
static volatile int  _usbh_dma_max_mem_used;
static volatile int  _dma_men_pool_used;

UDEV_T *g_udev_list;

#if (NVT_DCACHE_ON == 1)
    /* device address pool are placed in a non-cacheable region */
    NVT_NONCACHEABLE uint8_t  _dev_addr_pool[128];
    NVT_NONCACHEABLE static volatile int  _device_addr;
#else
    uint8_t  _dev_addr_pool[128];
    static volatile int  _device_addr;
#endif
static  int  _sidx = 0;

/*--------------------------------------------------------------------------*/
/*   Memory alloc/free recording                                            */
/*--------------------------------------------------------------------------*/

void usbh_memory_init(void)
{
    if (sizeof(TD_T) > HW_MEM_UNIT_SIZE)
    {
        USB_error("TD_T - MEM_POOL_UNIT_SIZE too small!\n");

        while (1);
    }

    if (sizeof(ED_T) > HW_MEM_UNIT_SIZE)
    {
        USB_error("ED_T - MEM_POOL_UNIT_SIZE too small!\n");

        while (1);
    }

    _usbh_hw_mem_used = 0L;
    _usbh_hw_max_mem_used = 0L;

    _usbh_dma_mem_used = 0L;
    _usbh_dma_max_mem_used = 0L;

    memset(_hw_unit_used, 0, sizeof(_hw_unit_used));
    _hw_mem_pool_used = 0;

    memset(_dma_unit_used, 0, sizeof(_dma_unit_used));
    _dma_men_pool_used = 0;

    _sidx = 0;

    g_udev_list = NULL;

    memset(_dev_addr_pool, 0, sizeof(_dev_addr_pool));
    _device_addr = 1;
}

uint32_t  usbh_memory_used(void)
{
    USB_debug("USB H/W memory: %d/%d, DMA memory: %d/%d\n", _hw_mem_pool_used, HW_MEM_UNIT_NUM,
              _dma_men_pool_used, DMA_MEM_UNIT_NUM);
    return _dma_men_pool_used;
}

static void  memory_counter(int size)
{
    _usbh_dma_mem_used += size;

    if (_usbh_dma_mem_used > _usbh_dma_max_mem_used)
        _usbh_dma_max_mem_used = _usbh_dma_mem_used;
}

void *usbh_alloc_mem(int size)
{
    int  i, start;
    int  found, wanted;
    void  *p;

    start = -1;
    found = 0;
    wanted = (size + DMA_MEM_UNIT_SIZE - 1) / DMA_MEM_UNIT_SIZE;

    for (i = 0; i < DMA_MEM_UNIT_NUM - wanted + 1; i++)
    {
        if (_dma_unit_used[i] == 0)
        {
            if (found == 0)
                start = i;

            found++;

            if (found >= wanted)
                break;
        }
        else
        {
            found = 0;
        }
    }

    if (found < wanted)
    {
        USB_error("%s failed to allocate %d KB!!! (%d / %d)\n", __func__,
                  size / 1024, _dma_men_pool_used, DMA_MEM_UNIT_NUM);
        return NULL;
    }

    /* Go allocate it */
    for (i = start; found > 0; i++, found--)
    {
        _dma_unit_used[i] = 1;
    }

    _dma_men_pool_used += wanted;


    memset(&_dma_mem_pool[start], 0, DMA_MEM_UNIT_SIZE * wanted);
    memory_counter(size);
    p = (void *)&_dma_mem_pool[start];
    return p;
}

int usbh_free_mem(void *p, int size)
{
    int i, start, wanted;
    uint32_t paddr, base;

    paddr = (uint32_t)(p);
    base = (uint32_t)(&_dma_mem_pool[0]);

    if ((paddr < base) || (paddr > base + (DMA_MEM_UNIT_NUM - 1) * DMA_MEM_UNIT_SIZE))
    {
        USB_error("%s - invalid DMA address 0x%x!\n", __func__, (uint32_t)paddr);
        return USBH_ERR_MEM_FREE_INVALID;
    }

    start = (paddr - base) / DMA_MEM_UNIT_SIZE;

    if ((uint32_t)&_dma_mem_pool[start] != paddr)
    {
        USB_error("%s paddr not block aligned: 0x%x\n", __func__, (uint32_t)paddr);
        return USBH_ERR_MEM_FREE_INVALID;
    }

    wanted = (size + DMA_MEM_UNIT_SIZE - 1) / DMA_MEM_UNIT_SIZE;

    if ((paddr + wanted * DMA_MEM_UNIT_SIZE) > (base + DMA_MEM_UNIT_NUM * DMA_MEM_UNIT_SIZE))
    {
        USB_error("%s - invalid DMA address 0x%x, size %d!\n", __func__, (uint32_t)paddr, size);
        return USBH_ERR_MEM_FREE_INVALID;
    }

    for (i = start; i < start + wanted; i++)
    {
        if (!_dma_unit_used[i])
            USB_error("%s warning - try to free an unused block %d!\n", __func__, i);

        _dma_unit_used[i] = 0;
    }

    _dma_men_pool_used -= wanted;

    memory_counter(0 - size);

    return USBH_OK;
}

/*--------------------------------------------------------------------------*/
/*   USB device allocate/free                                               */
/*--------------------------------------------------------------------------*/

UDEV_T *alloc_device(void)
{
    UDEV_T  *udev;

    udev = usbh_alloc_mem(sizeof(*udev));

    if (udev == NULL)
    {
        USB_error("alloc_device failed!\n");
        return NULL;
    }

    memset(udev, 0, sizeof(*udev));
    memory_counter(sizeof(*udev));
    udev->cur_conf = -1;                    /* must! used to identify the first SET CONFIGURATION */
    udev->next = g_udev_list;               /* chain to global device list */
    g_udev_list = udev;
    return udev;
}

void free_device(UDEV_T *udev)
{
    UDEV_T  *d;

    if (udev == NULL)
        return;

    if (udev->cfd_buff != NULL)
        usbh_free_mem(udev->cfd_buff, MAX_DESC_BUFF_SIZE);

    /*
     *  Remove it from the global device list
     */
    if (g_udev_list == udev)
    {
        g_udev_list = g_udev_list->next;
    }
    else
    {
        d = g_udev_list;

        while (d != NULL)
        {
            if (d->next == udev)
            {
                d->next = udev->next;
                break;
            }

            d = d->next;
        }
    }

    usbh_free_mem(udev, sizeof(*udev));

    memory_counter(-sizeof(*udev));
}

int  alloc_dev_address(void)
{
    _device_addr++;

    if (_device_addr >= 128)
        _device_addr = 1;

    while (1)
    {
        if (_dev_addr_pool[_device_addr] == 0)
        {
            _dev_addr_pool[_device_addr] = 1;
            return _device_addr;
        }

        _device_addr++;

        if (_device_addr >= 128)
            _device_addr = 1;
    }
}

void  free_dev_address(int dev_addr)
{
    if (dev_addr < 128)
        _dev_addr_pool[dev_addr] = 0;
}

/*--------------------------------------------------------------------------*/
/*   UTR (USB Transfer Request) allocate/free                               */
/*--------------------------------------------------------------------------*/

UTR_T *alloc_utr(UDEV_T *udev)
{
    UTR_T  *utr;

    utr = usbh_alloc_mem(sizeof(*utr));

    if (utr == NULL)
    {
        USB_error("alloc_utr failed!\n");
        return NULL;
    }

    memory_counter(sizeof(*utr));
    memset(utr, 0, sizeof(*utr));
    utr->udev = udev;
    mem_debug("[ALLOC] [UTR] - 0x%x\n", (int)utr);
    return utr;
}

void free_utr(UTR_T *utr)
{
    if (utr == NULL)
        return;

    mem_debug("[FREE] [UTR] - 0x%x\n", (int)utr);
    usbh_free_mem(utr, sizeof(*utr));
    memory_counter(0 - (int)sizeof(*utr));
}

/*--------------------------------------------------------------------------*/
/*   OHCI ED allocate/free                                                  */
/*--------------------------------------------------------------------------*/

ED_T *alloc_ohci_ED(void)
{
    int    i;
    ED_T   *ed;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if (_hw_unit_used[i] == 0)
        {
            _hw_unit_used[i] = 1;
            _hw_mem_pool_used++;
            ed = (ED_T *)&_hw_mem_pool[i];
            memset(ed, 0, sizeof(*ed));
            mem_debug("[ALLOC] [ED] - 0x%x\n", (int)ed);
            return ed;
        }
    }

    USB_error("alloc_ohci_ED failed!\n");
    return NULL;
}

void free_ohci_ED(ED_T *ed)
{
    int      i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)ed)
        {
            mem_debug("[FREE]  [ED] - 0x%x\n", (int)ed);
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            return;
        }
    }

    USB_debug("free_ohci_ED - not found! (ignored in case of multiple UTR)\n");
}

/*--------------------------------------------------------------------------*/
/*   OHCI TD allocate/free                                                  */
/*--------------------------------------------------------------------------*/
TD_T *alloc_ohci_TD(UTR_T *utr)
{
    int    i;
    TD_T   *td;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if (_hw_unit_used[i] == 0)
        {
            _hw_unit_used[i] = 1;
            _hw_mem_pool_used++;
            td = (TD_T *)&_hw_mem_pool[i];

            memset(td, 0, sizeof(*td));
            td->utr = utr;
            mem_debug("[ALLOC] [TD] - 0x%x\n", (int)td);
            return td;
        }
    }

    USB_error("alloc_ohci_TD failed!\n");
    return NULL;
}

void free_ohci_TD(TD_T *td)
{
    int   i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)td)
        {
            mem_debug("[FREE]  [TD] - 0x%x\n", (int)td);
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            return;
        }
    }

    USB_error("free_ohci_TD - not found!\n");
}

/*--------------------------------------------------------------------------*/
/*   EHCI QH allocate/free                                                  */
/*--------------------------------------------------------------------------*/
QH_T *alloc_ehci_QH(void)
{
    int    i;
    QH_T   *qh = NULL;

    for (i = (_sidx + 1) % HW_MEM_UNIT_NUM; i != _sidx; i = (i + 1) % HW_MEM_UNIT_NUM)
    {
        if (_hw_unit_used[i] == 0)
        {
            _hw_unit_used[i] = 1;
            _sidx = i;
            _hw_mem_pool_used++;
            qh = (QH_T *)&_hw_mem_pool[i];
            memset(qh, 0, sizeof(*qh));
            mem_debug("[ALLOC] [QH] - 0x%x\n", (int)qh);
            break;
        }
    }

    if (qh == NULL)
    {
        USB_error("alloc_ehci_QH failed!\n");
        return NULL;
    }

    qh->Curr_qTD        = QTD_LIST_END;
    qh->OL_Next_qTD     = QTD_LIST_END;
    qh->OL_Alt_Next_qTD = QTD_LIST_END;
    qh->OL_Token        = QTD_STS_HALT;
    return qh;
}

void free_ehci_QH(QH_T *qh)
{
    int      i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)qh)
        {
            mem_debug("[FREE]  [QH] - 0x%x\n", (int)qh);
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            return;
        }
    }

    USB_debug("free_ehci_QH - not found! (ignored in case of multiple UTR)\n");
}

/*--------------------------------------------------------------------------*/
/*   EHCI qTD allocate/free                                                 */
/*--------------------------------------------------------------------------*/
qTD_T *alloc_ehci_qTD(UTR_T *utr)
{
    int     i;
    qTD_T   *qtd;

    for (i = (_sidx + 1) % HW_MEM_UNIT_NUM; i != _sidx; i = (i + 1) % HW_MEM_UNIT_NUM)
    {
        if (_hw_unit_used[i] == 0)
        {
            _hw_unit_used[i] = 1;
            _sidx = i;
            _hw_mem_pool_used++;
            qtd = (qTD_T *)&_hw_mem_pool[i];

            memset(qtd, 0, sizeof(*qtd));
            qtd->Next_qTD     = QTD_LIST_END;
            qtd->Alt_Next_qTD = QTD_LIST_END;
            qtd->Token        = 0x11197B7F;//0x1197B3F; // QTD_STS_HALT;  visit_qtd() will not remove a qTD with this mark. It means the qTD still not ready for transfer.
            qtd->utr = utr;
            mem_debug("[ALLOC] [qTD] - 0x%x\n", (int)qtd);
            return qtd;
        }
    }

    USB_error("alloc_ehci_qTD failed!\n");
    return NULL;
}

void free_ehci_qTD(qTD_T *qtd)
{
    int   i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)qtd)
        {
            mem_debug("[FREE]  [qTD] - 0x%x\n", (int)qtd);
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            return;
        }
    }

    USB_error("free_ehci_qTD 0x%x - not found!\n", (int)qtd);
}

/*--------------------------------------------------------------------------*/
/*   EHCI iTD allocate/free                                                 */
/*--------------------------------------------------------------------------*/
iTD_T *alloc_ehci_iTD(void)
{
    int     i;
    iTD_T   *itd;

    for (i = (_sidx + 1) % HW_MEM_UNIT_NUM; i != _sidx; i = (i + 1) % HW_MEM_UNIT_NUM)
    {
        if (i + 2 >= HW_MEM_UNIT_NUM)
            continue;

        if ((_hw_unit_used[i] == 0) && (_hw_unit_used[i + 1] == 0))
        {
            _hw_unit_used[i] = _hw_unit_used[i + 1] = 1;
            _sidx = i + 1;
            _hw_mem_pool_used += 2;
            itd = (iTD_T *)&_hw_mem_pool[i];
            memset(itd, 0, sizeof(*itd));
            mem_debug("[ALLOC] [iTD] - 0x%x\n", (int)itd);
            return itd;
        }
    }

    USB_error("alloc_ehci_iTD failed!\n");
    return NULL;
}

void free_ehci_iTD(iTD_T *itd)
{
    int   i;

    for (i = 0; i + 1 < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)itd)
        {
            mem_debug("[FREE]  [iTD] - 0x%x\n", (int)itd);
            _hw_unit_used[i] = _hw_unit_used[i + 1] = 0;
            _hw_mem_pool_used -= 2;
            return;
        }
    }

    USB_error("free_ehci_iTD 0x%x - not found!\n", (int)itd);
}

/*--------------------------------------------------------------------------*/
/*   EHCI iTD allocate/free                                                 */
/*--------------------------------------------------------------------------*/
siTD_T *alloc_ehci_siTD(void)
{
    int     i;
    siTD_T  *sitd;

    for (i = (_sidx + 1) % HW_MEM_UNIT_NUM; i != _sidx; i = (i + 1) % HW_MEM_UNIT_NUM)
    {
        if (_hw_unit_used[i] == 0)
        {
            _hw_unit_used[i] = 1;
            _sidx = i;
            _hw_mem_pool_used ++;
            sitd = (siTD_T *)&_hw_mem_pool[i];
            memset(sitd, 0, sizeof(*sitd));
            mem_debug("[ALLOC] [siTD] - 0x%x\n", (int)sitd);
            return sitd;
        }
    }

    USB_error("alloc_ehci_siTD failed!\n");
    return NULL;
}

void free_ehci_siTD(siTD_T *sitd)
{
    int   i;

    for (i = 0; i < HW_MEM_UNIT_NUM; i++)
    {
        if ((uint32_t)&_hw_mem_pool[i] == (uint32_t)sitd)
        {
            mem_debug("[FREE]  [siTD] - 0x%x\n", (int)sitd);
            _hw_unit_used[i] = 0;
            _hw_mem_pool_used--;
            return;
        }
    }

    USB_error("free_ehci_siTD 0x%x - not found!\n", (int)sitd);
}

/// @endcond HIDDEN_SYMBOLS
