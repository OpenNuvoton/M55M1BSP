/**************************************************************************//**
 * @file     cdc_parser.c
 * @version  V1.00
 * @brief    USB Host CDC report descriptor parser.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "usbh_lib.h"
#include "usbh_cdc.h"

/// @cond HIDDEN_SYMBOLS

static int  cdc_parse_cs_interface(CDC_DEV_T *cdev, uint8_t *buffer, int size)
{
    CDC_IF_HDR_T    *cifd;
    int             parsed = 0;
    int             size_tmp = size;

    while (size_tmp > 0)
    {

        while (size_tmp >= (int)sizeof(DESC_HDR_T))
        {
            DESC_HDR_T      *header;
            header = (DESC_HDR_T *)&buffer[parsed];

            if (header->bLength < 2)
            {
                CDC_DBGMSG("Invalid descriptor length of %d\n", header->bLength);
                return -1;
            }

            if (header->bDescriptorType != CDC_CS_INTERFACE)
            {
                return parsed;
            }

            cifd = (CDC_IF_HDR_T *)header;

            CDC_DBGMSG("CS_INTERFACE: 0x%x, ", cifd->bDescriptorSubtype);

            switch (cifd->bDescriptorSubtype)
            {
                case CDC_DT_HDR_FUNC:
                    CDC_DBGMSG("Header Functional\n");
                    break;

                case CDC_DT_CALL_MANAGE:
                    CDC_DBGMSG("Call Management\n");
                    break;

                case CDC_DT_ABS_CTRL:
                    CDC_DBGMSG("Abstract Control Management\n");
                    break;

                case CDC_DT_LINE_MANAGE:
                    CDC_DBGMSG("Direct Line Management\n");
                    break;

                case CDC_DT_TEL_RINGER:
                    CDC_DBGMSG("Telephone Ringer\n");
                    break;

                case CDC_DT_TEL_OPER_MODES:
                    CDC_DBGMSG("Telephone Operational Modes\n");
                    break;

                case CDC_DT_CALL_LINE_CAP:
                    CDC_DBGMSG("Telephone Call and Line State Reporting Capabilities\n");
                    break;

                case CDC_DT_UNION:
                    CDC_DBGMSG("Union Functional\n");

                    if (cifd->bLength >= 5)
                    {
                        cdev->ifnum_data = cifd->payload[1];
                    }

                    if (cifd->bLength >= 6)
                    {
                        CDC_DBGMSG("Union Functional length %d, not supported!\n", cifd->bLength);
                    }

                    break;

                case CDC_DT_COUNTRY_SEL:
                    CDC_DBGMSG("Country Selection\n");
                    break;

                case CDC_DT_USB_TERMINAL:
                    CDC_DBGMSG("USB Terminal\n");
                    break;

                case CDC_DT_NET_CHANNEL:
                    CDC_DBGMSG("Network Channel Terminal\n");
                    break;

                case CDC_DT_PROTO_UNIT:
                    CDC_DBGMSG("Protocol Unit\n");
                    break;

                case CDC_DT_EXTENT_UNIT:
                    CDC_DBGMSG("Extension Unit\n");
                    break;

                case CDC_DT_MULTI_CHANNEL:
                    CDC_DBGMSG("Multi-Channel Management\n");
                    break;

                case CDC_DT_CAPI_CTRL:
                    CDC_DBGMSG("CAPI Control Management\n");
                    break;

                case CDC_DT_ETHERNET_FUNC:
                    CDC_DBGMSG("Ethernet Networking Functional\n");
                    break;

                case CDC_DT_ATM_FUNC:
                    CDC_DBGMSG("ATM Networking Functional\n");
                    break;

                default:
                    CDC_DBGMSG("Unknown Functional Descriptor: 0x%x\n", cifd->bDescriptorSubtype);
                    break;
            }

            parsed += header->bLength;
            size_tmp -= header->bLength;
        }
    }   /* end of while */

    return parsed;
}

int  cdc_config_parser(CDC_DEV_T *cdev)
{
    UDEV_T          *udev = cdev->udev;
    DESC_CONF_T     *config;
    DESC_HDR_T      *header;
    uint8_t         *cfg_buff;
    int             parsed_len;
    int             size;

    config = (DESC_CONF_T *)udev->cfd_buff;
    cfg_buff = (uint8_t *)config;
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    while (size >= (int)sizeof(DESC_HDR_T))
    {
        header = (DESC_HDR_T *)&cfg_buff[parsed_len];

        if ((header->bLength > size) || (header->bLength < 2))
        {
            CDC_DBGMSG("Error - invalid descriptor length of %d\n", header->bLength);
            return USBH_ERR_NOT_SUPPORTED;
        }

        /*
         *  Is the interface descriptor of this CDC device?
         */
        if (header->bDescriptorType == USB_DT_INTERFACE)
        {
            const DESC_IF_T       *ifd;
            ifd = (const DESC_IF_T *)header;

            if (ifd->bInterfaceNumber == cdev->iface_cdc->if_num)
            {
                parsed_len += header->bLength;
                size -= header->bLength;
                break;
            }
        }

        parsed_len += header->bLength;
        size -= header->bLength;
    }   /* end of while */

    /*------------------------------------------------------------------*/
    /*  Parsing all follwoing CDC class interface descriptors           */
    /*------------------------------------------------------------------*/

    while (size >= (int)sizeof(DESC_HDR_T))
    {
        header = (DESC_HDR_T *)&cfg_buff[parsed_len];

        if ((header->bLength > size) || (header->bLength < 2))
        {
            CDC_DBGMSG("Error - invalid descriptor length of %d\n", header->bLength);
            return USBH_ERR_NOT_SUPPORTED;
        }

        /*
         *  Is a class interface descriptor?
         */
        if (header->bDescriptorType != CDC_CS_INTERFACE)
        {
            break;
        }

        int             result;
        result = cdc_parse_cs_interface(cdev, &cfg_buff[parsed_len], size);

        if (result < 0)
        {
            return result;
        }

        parsed_len += result;
        size -= result;
    }   /* end of while */

    CDC_DBGMSG("CDC ifnum_cdc = %d\n", cdev->iface_cdc->if_num);

    if (cdev->iface_data)
    {
        CDC_DBGMSG("CDC ifnum_data = %d\n", cdev->iface_data->if_num);
    }

    return 0;
}

/// @endcond HIDDEN_SYMBOLS
