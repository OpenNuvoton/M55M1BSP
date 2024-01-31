/**************************************************************************//**
 * @file     ohci.c
 * @version  V1.10
 * @brief   USB Host library OHCI (USB 1.1) host controller driver.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "hub.h"
#include "ohci.h"

/// @cond HIDDEN_SYMBOLS
#define _ohci_port      (0UL)
#define _ohci           _ohci0
#define ohci_driver     ohci0_driver

#define OHCI_IRQHandler   USBH0_IRQHandler

#define ENABLE_OHCI_IRQ   ENABLE_OHCI0_IRQ
#define DISABLE_OHCI_IRQ  DISABLE_OHCI0_IRQ

static uint16_t  port_mask = 0x0001;

#include "ohci.c"

HC_DRV_T  ohci0_driver =
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
    ohci_rh_polling,         /* rthub_polling      */
};
/// @endcond HIDDEN_SYMBOLS