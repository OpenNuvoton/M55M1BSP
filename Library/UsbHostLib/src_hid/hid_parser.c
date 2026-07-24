/**************************************************************************//**
 * @file     hid_parser.c
 * @version  V1.00
 * @brief    USB Host HID report descriptor parser.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "usbh_lib.h"
#include "usbh_hid.h"

/// @cond HIDDEN_SYMBOLS
int       _data_usage_cnt;

static int hid_parse_item(HID_DEV_T *hdev, uint8_t *buff);

#if ENABLE_DBG_MSG
struct string_table
{
    char        string[32];
    uint8_t     code;
};

static const struct string_table usage_page_list[] =
{
    "Generic Desktop",      UP_GENERIC_DESKTOP,
    "Simulation Controls",  UP_SIMULATION_CONTROLS,
    "VR Controls",          UP_VR_CONTROLS,
    "Sport Controls",       UP_SPORT_CONTROLS,
    "Game Controls",        UP_GAME_CONTROLS,
    "KeyCode",              UP_KEYCODE,
    "LEDs",                 UP_LEDS,
    "Button",               UP_BUTTON,
    "Digitizer",            UP_DIGITIZER,
    "Bar Code Scanner",     UP_BARCODE_SCANNER,
};

static const struct string_table desktop_page_list[] =
{
    "Pointer",              USAGE_ID_POINTER,
    "Mouse",                USAGE_ID_MOUSE,
    "Joystick",             USAGE_ID_JOYSTICK,
    "Game Pad",             USAGE_ID_GAMEPAD,
    "Keyboard",             USAGE_ID_KEYBOARD,
    "Keypad",               USAGE_ID_KEYPAD,
    "X",                    USAGE_ID_X,
    "Y",                    USAGE_ID_Y,
    "Z",                    USAGE_ID_Z,
    "Rx",                   0x33,
    "Ry",                   0x34,
    "Rz",                   0x35,
    "Slider",               0x36,
    "Dial",                 0x37,
    "Wheel",                USAGE_ID_WHEEL,
};

#endif

/*
 * Varibles used on parsing HID report descriptor
 */
static RP_INFO_T   _rp_info;      /* describing the current report */

static void print_usage_page(void)
{
#if ENABLE_DBG_MSG
    int   i;

    for (i = 0; i < sizeof(usage_page_list) / sizeof(struct string_table); i++)
    {
        if (usage_page_list[i].code == _rp_info.usage_page)
        {
            USB_debug("(%s)", usage_page_list[i].string);
            return;
        }
    }

    USB_debug("(?? - 0x%x)", _rp_info.usage_page);
#endif
}

static void print_usage(uint8_t usage)
{
#if ENABLE_DBG_MSG
    int   i, count;
    struct string_table  *p;

    if (_rp_info.usage_page == UP_GENERIC_DESKTOP)
    {
        count = sizeof(desktop_page_list) / sizeof(struct string_table);
        p = (struct string_table *)&desktop_page_list[0];
    }
    else
    {
        return;
    }

    for (i = 0; i < count; i++, p++)
    {
        if (p->code == usage)
        {
            USB_debug("(%s)", p->string);
            return;
        }
    }

    USB_debug("(?? - 0x%x)", usage);
#else
    NVT_UNUSED(usage);
#endif
}

static void read_main_item_status(const uint8_t *buff)
{
    HID_DBGMSG("(");

    if (buff[0] & 0x01U)
    {
        _rp_info.status.constant = 1;
        _rp_info.status.variable = 0;
        HID_DBGMSG("Constant ");
    }

    if (buff[0] & 0x02U)
    {
        _rp_info.status.constant = 0;
        _rp_info.status.variable = 1;
        HID_DBGMSG("Variable ");
    }

    if (buff[0] & 0x04U)
    {
        _rp_info.status.relative = 1;
        HID_DBGMSG("Relative ");
    }

    if (buff[0] & 0x08U)
    {
        _rp_info.status.wrap = 1;
        HID_DBGMSG("Wrap ");
    }

    if (buff[0] & 0x10U)
    {
        _rp_info.status.non_linear = 1;
        HID_DBGMSG("Non-linear ");
    }

    if (buff[0] & 0x20U)
    {
        _rp_info.status.no_preferred = 1;
        HID_DBGMSG("Not-prefered ");
    }

    if (buff[0] & 0x40U)
    {
        _rp_info.status.USBNULL_state = 1;
        HID_DBGMSG("USBNULL-state ");
    }

    if (buff[0] & 0x80U)
    {
        _rp_info.status.is_volatile = 1;
        HID_DBGMSG("Volatile ");
    }

    if (buff[1] & 0x01U)
    {
        _rp_info.status.buffered_bytes = 1;
        HID_DBGMSG("Buffered-bytes ");
    }

    HID_DBGMSG(")");
}

/**
 *  @brief  Parse report descriptor and get information from descriptors.
 *  @param[in]  hdev    HID device
 *  @param[in]  iface   HID class interface
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otheriwse  Error occurred
 */
int hid_parse_report_descriptor(HID_DEV_T *hdev, IFACE_T *iface)
{
    UDEV_T         *udev = iface->udev;
    const DESC_CONF_T    *config;
    const DESC_IF_T      *ifd = USBNULL;
    const DESC_HID_T     *hidd;
    uint8_t        *cfg_buff;
    uint8_t        *desc_buff;
    int            desc_buff_len;
    int            desc_offset;
    int            parsed_len;
    int            remain_len;
    int            size;

    HID_DBGMSG("HID interface %d parsing report descriptor...\n", iface->if_num);

    (void)memset(&_rp_info, 0, sizeof(_rp_info));
    _data_usage_cnt = 0;

    hdev->rpd.has_report_id = 0;

    cfg_buff = udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;

    /* step over configuration descritpor */
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Find the Interface Descriptor of this HID interface                               */
    /*------------------------------------------------------------------------------------*/
    while (size >= (int)sizeof(DESC_IF_T))
    {
        ifd = (const DESC_IF_T *)&cfg_buff[parsed_len];

        if ((ifd->bDescriptorType == USB_DT_INTERFACE) && (ifd->bInterfaceNumber == iface->if_num) &&
                (ifd->bInterfaceClass == USB_CLASS_HID))
        {
            break;
        }

        if (ifd->bLength == 0)
        {
            return -1;
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    if (size < (int)sizeof(DESC_IF_T))
    {
        HID_ERRMSG("Can't find the HID interface!\n");
        return HID_RET_PARSING;
    }

    parsed_len += ifd->bLength;
    size -= ifd->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Continue to find the subsequent HID Descriptor                                    */
    /*------------------------------------------------------------------------------------*/
    while (size >= (int)sizeof(DESC_HID_T))
    {
        hidd = (const DESC_HID_T *)&cfg_buff[parsed_len];

        if ((hidd->bDescriptorType == HID_DESCRIPTOR_TYPE) &&
                (hidd->bRPDescType == REPORT_DESCRIPTOR_TYPE))
        {
            break;
        }

        if (hidd->bLength == 0)
        {
            return HID_RET_PARSING;
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    if (size < (int)sizeof(DESC_HID_T))
    {
        HID_ERRMSG("Can't find the HID interface!\n");
        return HID_RET_PARSING;
    }

    hidd = (const DESC_HID_T *)&cfg_buff[parsed_len];

    HID_DBGMSG("[HID Descriptor]\n");
    HID_DBGMSG("bLength = %d\n", hidd->bLength);
    HID_DBGMSG("bDescriptorType = 0x%x\n", hidd->bDescriptorType);
    HID_DBGMSG("bcdHID = 0x%x\n", hidd->bcdHID);
    HID_DBGMSG("bCountryCode = 0x%x\n", hidd->bCountryCode);
    HID_DBGMSG("bNumDescriptors = %d\n", hidd->bNumDescriptors);
    HID_DBGMSG("bRPDescType = 0x%x\n", hidd->bRPDescType);
    HID_DBGMSG("wDescriptorLength = %d\n", hidd->wDescriptorLength);

    HID_DBGMSG("Report descriptor found, length=%d. %d\n", hidd->wDescriptorLength, hidd->bLength);

    desc_buff_len = hidd->wDescriptorLength + 8;
    desc_buff = (uint8_t *)usbh_alloc_mem(desc_buff_len);

    remain_len = usbh_hid_get_report_descriptor(hdev, desc_buff, desc_buff_len);

    if (remain_len <= 0)
    {
        (void)usbh_free_mem(desc_buff, desc_buff_len);
        return remain_len;
    }

    //HID_DBGMSG("\nDump report descriptor =>\n");
    //dump_buff_hex(desc_buff, remain_len);

    /*------------------------------------------------------------------------------------*/
    /*  Parsing items                                                                     */
    /*------------------------------------------------------------------------------------*/
    desc_offset = 0;

    while (remain_len > 0)
    {
        size = hid_parse_item(hdev, &desc_buff[desc_offset]);

        //printf("size = %d/%d\n", size, remain_len);
        if (size <= 0)
        {
            (void)usbh_free_mem(desc_buff, desc_buff_len);
            return HID_RET_PARSING;
        }

        desc_offset += size;
        remain_len -= size;
    }

    (void)usbh_free_mem(desc_buff, desc_buff_len);

    /*------------------------------------------------------------------------------------*/
    /*  For keyboard device, turn on all LEDs for 0.5 seconds and then turn off.          */
    /*------------------------------------------------------------------------------------*/
    if ((hdev->bSubClassCode == HID_SUBCLASS_BOOT_DEVICE) && (hdev->bProtocolCode == HID_PROTOCOL_KEYBOARD))
    {
        const RP_INFO_T   *report;

        for (report = hdev->rpd.report; report != USBNULL; report = report->next)
        {
            if ((report->usage_page == UP_LEDS) && (report->report_size == 1U) && report->status.variable)
            {
                uint8_t  i;
                uint8_t ret;
                uint8_t leds = 0;

                for (i = 0; (i < 8U) && (i < report->report_count); i++)
                {
                    leds = (leds << 1) | 0x1U;
                }

                /* turn-on keyboard NumLock, CapsLock, ScrollLock LEDs */
                ret = usbh_hid_set_report(hdev, RT_OUTPUT, 0, &leds, 1);

                if (ret != 1U)
                {
                    HID_ERRMSG("Failed to turn on LEDs! 0x%x, %d\n", leds, ret);
                }
                else
                {
                    delay_us(500000);       /* delay 0.5 conds */

                    /* turn-off all LEDs */
                    leds = 0x00;
                    ret = usbh_hid_set_report(hdev, RT_OUTPUT, 0, &leds, 1);

                    if (ret != 1U)
                    {
                        HID_ERRMSG("Failed to turn off LEDs! %d\n", ret);
                    }
                }
            }
        }
    }

    return 0;
}

static int hid_add_report(HID_DEV_T *hdev, uint8_t type)
{
    RP_INFO_T   *report;
    RP_INFO_T   *p;

    report = (RP_INFO_T *)usbh_alloc_mem(sizeof(RP_INFO_T));

    if (report == USBNULL)
    {
        HID_ERRMSG("hid_add_report allocate memory failed!!\n");
        return USBH_ERR_MEMORY_OUT;
    }

    (void)memcpy(report, &_rp_info, sizeof(RP_INFO_T));
    report->type = type;

    HID_DBGMSG("\nCreate a report. %d x %d (%d)\n", report->report_count, report->report_size, report->report_id);

    if (hdev->rpd.report == USBNULL)
    {
        hdev->rpd.report = report;
    }
    else
    {
        p = hdev->rpd.report;

        while (p->next  != USBNULL)
        {
            p = p->next;
        }

        p->next = report;
    }

    return 0;
}

static signed int hid_read_item_value(uint8_t bSize, const uint8_t *buff)
{
    if (bSize == 1U)
    {
        return (signed char)buff[0];
    }
    else if (bSize == 2U)
    {
        return (signed short)(uint16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8U));
    }
    else if (bSize == 4U)
    {
        return (signed int)(uint32_t)((uint32_t)buff[0] | ((uint32_t)buff[1] << 8U) | ((uint32_t)buff[2] << 16U) | ((uint32_t)buff[3] << 24U));
    }
    else
    {
        return 0;
    }
}

static int hid_parse_item(HID_DEV_T *hdev, uint8_t *buff)
{
    static uint8_t   _data_usages[16];
    uint8_t     bTag;
    uint8_t     bSize;
    uint8_t     tag;
    int         item_len;
    int         i;

    bTag  = (buff[0] >> 4) & 0xFU;
    //bType = (buff[0] >> 2) & 0x3;
    bSize = buff[0] & 0x3U;
    tag = (buff[0] & 0xFCU);

    if (bTag == 0xFU)
    {
        bSize = buff[1];
        item_len = (int)bSize + 3;
    }
    else
    {
        if (bSize == 0x3U)
        {
            bSize = 4;
        }

        item_len = (int)bSize + 1;
    }

#if ENABLE_DBG_MSG

    for (i = 0; i < item_len; i++)
    {
        USB_debug("%02x ", buff[i]);
    }

    USB_debug("- ");
#endif

    switch (tag)
    {
        /*------------------------------------------------------------------------------------*/
        /*  Main Item Tags                                                                    */
        /*------------------------------------------------------------------------------------*/

        case TAG_INPUT:
            HID_DBGMSG("Input ");
            read_main_item_status(&buff[1]);

            if (_data_usage_cnt > 0)
            {
                int  report_count = _rp_info.report_count;

                for (i = 0; i < _data_usage_cnt; i++)
                {
                    _rp_info.report_count = 1;
                    _rp_info.data_usage = _data_usages[i];

                    if (hid_add_report(hdev, TAG_INPUT) != 0)
                    {
                        return USBH_ERR_MEMORY_OUT;
                    }

                    report_count--;
                }

                _rp_info.report_count = report_count;
                _rp_info.data_usage = 0;
                _data_usage_cnt = 0;
            }

            if (_rp_info.report_count > 0U)
            {
                if (hid_add_report(hdev, TAG_INPUT) != 0)
                {
                    return USBH_ERR_MEMORY_OUT;
                }
            }

            break;

        case TAG_OUTPUT:
            HID_DBGMSG("Output ");
            read_main_item_status(&buff[1]);

            if (_rp_info.report_count > 0U)
            {
                if (hid_add_report(hdev, TAG_OUTPUT) != 0)
                {
                    return USBH_ERR_MEMORY_OUT;
                }
            }

            break;

        case TAG_FEATURE:
            HID_DBGMSG("Feature ");
            read_main_item_status(&buff[1]);
            break;

        case TAG_COLLECTION:
            HID_DBGMSG("Collection ");

            if (buff[1] == 0x00U)
            {
                HID_DBGMSG("Physical");
            }
            else if (buff[1] == 0x01U)
            {
                HID_DBGMSG("Application");
            }
            else if (buff[1] == 0x02U)
            {
                HID_DBGMSG("Logical");
            }
            else
            {
                ///  Other collection types are reserved for future use, and should be treated as "Application Collection".
            }

            break;

        case TAG_END_COLLECTION:
            HID_DBGMSG("End Collection");
            break;

        /*------------------------------------------------------------------------------------*/
        /*  Global Item Tags                                                                  */
        /*------------------------------------------------------------------------------------*/

        case TAG_USAGE_PAGE:
            HID_DBGMSG("Usage Page ");
            _rp_info.usage_page = buff[1];
            print_usage_page();
            break;

        case TAG_LOGICAL_MIN:
            _rp_info.logical_min = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Logical Minimum (%d)", _rp_info.logical_min);
            break;

        case TAG_LOGICAL_MAX:
            _rp_info.logical_max = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Logical Maximum (%d)", _rp_info.logical_max);
            break;

        case TAG_PHYSICAL_MIN:
            _rp_info.physical_min = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Physical Minimum (%d)", _rp_info.physical_min);
            break;

        case TAG_PHYSICAL_MAX:
            _rp_info.physical_max = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Physical Maximum (%d)", _rp_info.physical_max);
            break;

        case TAG_UNIT_EXPONENT:
            _rp_info.unit_exponent = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Unit Exponent (%d)", _rp_info.unit_exponent);
            break;

        case TAG_UNIT:
            _rp_info.unit = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Unit (%d)", _rp_info.unit);
            break;

        case TAG_REPORT_SIZE:
            _rp_info.report_size = buff[1];
            HID_DBGMSG("Report Size (%d)", _rp_info.report_size);
            break;

        case TAG_REPORT_ID:
            _rp_info.report_id = buff[1];
            hdev->rpd.has_report_id = 1;
            HID_DBGMSG("Report ID (%d)", _rp_info.report_id);
            break;

        case TAG_REPORT_COUNT:
            _rp_info.report_count = buff[1];
            HID_DBGMSG("Report Count (%d)", _rp_info.report_count);
            break;

        case TAG_PUSH:
            HID_DBGMSG("PUSH");
            break;

        case TAG_POP:
            HID_DBGMSG("POP");
            break;

        /*------------------------------------------------------------------------------------*/
        /*  Local Item Tags                                                                   */
        /*------------------------------------------------------------------------------------*/

        case TAG_USAGE:
            if ((buff[1] == USAGE_ID_X) || (buff[1] == USAGE_ID_Y) || (buff[1] == USAGE_ID_WHEEL))
            {
                _data_usages[_data_usage_cnt] = buff[1];    /* interested usages */
                _data_usage_cnt++;
            }
            else
            {
                _rp_info.app_usage = buff[1];
            }

            HID_DBGMSG("Usage ");
            print_usage(buff[1]);
            break;

        case TAG_USAGE_MIN:
            _rp_info.usage_mim = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Usage Mimimum (%d)", _rp_info.usage_mim);
            break;

        case TAG_USAGE_MAX:
            _rp_info.usage_max = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Usage Maximum (%d)", _rp_info.usage_max);
            break;

        case TAG_DESIGNATOR_INDEX:
            _rp_info.designator_index = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Designator Index (%d)", _rp_info.designator_index);
            break;

        case TAG_DESIGNATOR_MIN:
            _rp_info.designator_min = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Designator Minimum (%d)", _rp_info.designator_min);
            break;

        case TAG_DESIGNATOR_MAX:
            _rp_info.designator_max = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("Designator Maximum (%d)", _rp_info.designator_max);
            break;

        case TAG_STRING_INDEX:
            _rp_info.string_index = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("String Index (%d)", _rp_info.string_index);
            break;

        case TAG_STRING_MIN:
            _rp_info.string_min = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("String Minimum (%d)", _rp_info.string_min);
            break;

        case TAG_STRING_MAX:
            _rp_info.string_max = hid_read_item_value(bSize, &buff[1]);
            HID_DBGMSG("String Maximum (%d)", _rp_info.string_max);
            break;

        case TAG_DELIMITER:
            HID_DBGMSG("Delimiter");
            break;

        default:
            HID_DBGMSG("Unknow tag: 0x%x\n", tag);
            break;
    }

    HID_DBGMSG("\n");

    return item_len;
}

int hid_parse_keyboard_reports(HID_DEV_T *hdev, const uint8_t *data, int data_len)
{
    RP_INFO_T   *report;
    int         i;
    int         bit;
    int         byte_idx = 0;
    int         bit_idx = 0;
    int         has_kbd_event = 0;
    int         report_id = 0;
    static KEYBOARD_EVENT_T  _keyboard_event;

    (void)memset(&_keyboard_event, 0, sizeof(_keyboard_event));
    _keyboard_event.lock_state = hdev->rpd.lock_state;

    /*
     *  Does this device use report ID?
     */
    if (hdev->rpd.has_report_id)
    {
        report_id = data[0];
        bit_idx = 8;
        byte_idx = 1;
    }

    for (report = hdev->rpd.report; report != USBNULL; report = report->next)
    {
        if (hdev->rpd.has_report_id && (report->report_id != (uint8_t)report_id))
        {
            continue;
        }

        if (report->type != TAG_INPUT)
        {
            continue;
        }

        /*----------------------------------------------------------------------*/
        /*  Extract keyboard report; only KeyCode reports are interested        */
        /*----------------------------------------------------------------------*/
        if ((report->usage_page == UP_KEYCODE) && (report->app_usage == USAGE_ID_KEYBOARD))
        {
            if ((report->report_size != (uint8_t)1) && (report->report_size != (uint8_t)8))
            {
                /* unlikely! seems violate HID spec. */
                HID_ERRMSG("Keycode report size %d is not supported!\n", report->report_size);
                return USBH_ERR_NOT_SUPPORTED;
            }

            if (report->report_size == (uint8_t)1)
            {
                for (i = 0; i < (int)report->report_count; i++)
                {
                    uint32_t usage_val = 0;
                    bit = (data[byte_idx] >> ((uint32_t)bit_idx % 8U)) & 0x1U;
                    usage_val |= ((uint32_t)bit << (uint32_t)i);

                    if (bit_idx < 8)            /* is in the first byte         */
                    {
                        _keyboard_event.modifier |= usage_val;
                    }
                    else if (bit_idx < 16)      /* is in the second byte (reserved)  */
                    {
                    }
                    else
                    {
                        if (bit_idx < (8 * 8))
                        {
                            _keyboard_event.keycode[(bit_idx - 16) / 8] |= usage_val;
                        }
                    }

                    bit_idx++;
                }

                byte_idx = (bit_idx / 8);
            }
            else   /* report->report_size == 8 */
            {
                for (i = 0; i < (int)report->report_count; i++)
                {
                    if (byte_idx == 0)
                    {
                        _keyboard_event.modifier = data[byte_idx];
                    }
                    else if (byte_idx == 1)
                    {
                        /* reserved byte */
                    }
                    else
                    {
                        if (byte_idx < 8)
                        {
                            _keyboard_event.keycode[byte_idx - 2] = data[byte_idx];
                        }
                    }

                    byte_idx++;
                }
            }

            has_kbd_event = 1;
        }
        else
        {
            /* not interested, just skip it */
            bit_idx += (int)report->report_size * (int)report->report_count;
            byte_idx = (bit_idx / 8);
        }

        if (byte_idx >= data_len)
        {
            break;
        }
    }

    /* Get the keyboard event callback function registered to HID class driver */
    static HID_KEYBOARD_FUNC *_ptr_keyboard_callback;
    _ptr_keyboard_callback = usbh_hid_get_keyboard_callback();

    if ((has_kbd_event) && (_ptr_keyboard_callback != USBNULL))
    {
        uint8_t   pressed_lock_keys = 0;
        char      update_LEDs = 0;

        /*----------------------------------------------------------------------*/
        /*  Scan received key code sequence                                     */
        /*----------------------------------------------------------------------*/
        for (i = 0; i < 6; i++)
        {
            switch (_keyboard_event.keycode[i])
            {
                case KEYCODE_NUM_LOCK:
                    pressed_lock_keys |= STATE_MASK_NUM_LOCK;
                    break;

                case KEYCODE_CAPS_LOCK:
                    pressed_lock_keys |= STATE_MASK_CAPS_LOCK;
                    break;

                case KEYCODE_SCROLL_LOCK:
                    pressed_lock_keys |= STATE_MASK_SCROLL_LOCK;
                    break;

                case 0:         /* empty */
                case 1:         /* error */
                    break;

                default:
                    _keyboard_event.keycode[_keyboard_event.key_cnt] = _keyboard_event.keycode[i];
                    _keyboard_event.key_cnt++;
                    break;
            }
        }

        /*----------------------------------------------------------------------*/
        /*  Update lock keys (Num Lock, Caps Lock, Scroll Lock)                 */
        /*----------------------------------------------------------------------*/
        for (i = 0; i < 3; i++)
        {
            if ((pressed_lock_keys & (1U << (uint32_t)i)) && (!(hdev->rpd.last_pressed_lock_keys & (1U << (uint32_t)i))))
            {
                /*
                 * A lock key pressed and it is not pressed in the last time.
                 * It should be a press down of the lock key.
                 */
                _keyboard_event.lock_state ^= (1U << (uint32_t)i);  /* switch on/off the corresponding lock state */
                update_LEDs = 1;
            }
        }

        hdev->rpd.last_pressed_lock_keys = pressed_lock_keys;  /* record the lock key press state for next time. */
        hdev->rpd.lock_state = _keyboard_event.lock_state;

        if (update_LEDs)
        {
            (void)usbh_hid_set_report_non_blocking(hdev, RT_OUTPUT, 0, &_keyboard_event.lock_state, 1);
        }

        _ptr_keyboard_callback(hdev, &_keyboard_event);
    }

    return 0;
}

int hid_parse_mouse_reports(HID_DEV_T *hdev, const uint8_t *data, int data_len)
{
    int         byte_idx = 0;
    int         bit_idx = 0;
    RP_INFO_T   *report;
    int         i;
    int         bit;
    int         has_mouse_event = 0;
    int         report_id = 0;
    static MOUSE_EVENT_T  _mouse_event;

    (void)memset(&_mouse_event, 0, sizeof(_mouse_event));

    /*
     *  Does this device use report ID?
     */
    if (hdev->rpd.has_report_id)
    {
        report_id = data[0];
        bit_idx = 8;
        byte_idx = 1;
    }

    for (report = hdev->rpd.report; report != USBNULL; report = report->next)
    {
        if (hdev->rpd.has_report_id && (report->report_id != (uint8_t)report_id))
        {
            continue;
        }

        if (report->type != TAG_INPUT)
        {
            continue;
        }

        /*----------------------------------------------------------------------*/
        /*  Extract mouse button report                                         */
        /*----------------------------------------------------------------------*/
        if ((report->usage_page == UP_BUTTON) &&
                ((report->app_usage == USAGE_ID_MOUSE) || (report->app_usage == USAGE_ID_POINTER)))
        {
            /* Get button data */
            if (report->status.variable)
            {
                _mouse_event.button_cnt = report->report_count;

                for (i = 0; i < (int)report->report_count; i++)
                {
                    bit = (data[byte_idx] >> ((uint32_t)bit_idx % 8U)) & 0x1U;
                    _mouse_event.button_map |= ((uint32_t)bit << (uint32_t)i);
                    bit_idx += (int)report->report_size;
                    byte_idx = (bit_idx / 8);
                }
            }
            else
            {
                /* ignore constant padding bits */
                bit_idx += (int)report->report_count * (int)report->report_size;
                byte_idx = (bit_idx / 8);
            }

            has_mouse_event = 1;
        }

        /*----------------------------------------------------------------------*/
        /*  Extract mouse X, Y, and WHEEL reports                               */
        /*----------------------------------------------------------------------*/
        else if ((report->usage_page == UP_GENERIC_DESKTOP) &&
                 ((report->app_usage == USAGE_ID_MOUSE) || (report->app_usage == USAGE_ID_POINTER) ||
                  (report->data_usage == USAGE_ID_WHEEL)))
        {
            uint32_t   usage_val = 0;
            signed     s_val = 0;

            for (i = 0; i < (int)report->report_size; i++)
            {
                bit = (data[byte_idx] >> ((uint32_t)bit_idx % 8U)) & 0x1U;
                usage_val |= ((uint32_t)bit << (uint32_t)i);
                bit_idx++;
                byte_idx = (bit_idx / 8);
            }

            if (report->report_size <= 8U)
            {
                s_val = (signed char)usage_val;
            }
            else if (report->report_size <= 16U)
            {
                s_val = (signed short)usage_val;
            }
            else
            {
                // unlikely! seems violate HID spec. */
            }

            if (report->data_usage == USAGE_ID_X)
            {
                _mouse_event.X = s_val;
                _mouse_event.X_raw = usage_val;
                _mouse_event.X_bits = report->report_size;
                _mouse_event.axis_relative = report->status.relative;
                _mouse_event.axis_min = report->logical_min;
                _mouse_event.axis_max = report->logical_max;
            }
            else if (report->data_usage == USAGE_ID_Y)
            {
                _mouse_event.Y = s_val;
                _mouse_event.Y_raw = usage_val;
                _mouse_event.Y_bits = report->report_size;
            }
            else if (report->data_usage == USAGE_ID_WHEEL)
            {
                _mouse_event.wheel = s_val;
                _mouse_event.wheel_raw = usage_val;
                _mouse_event.wheel_bits = report->report_size;
                _mouse_event.wheel_relative = report->status.relative;
                _mouse_event.wheel_min = report->logical_min;
                _mouse_event.wheel_max = report->logical_max;
            }
            else
            {
                // unlikely! seems violate HID spec. */
            }

            has_mouse_event = 1;
        }
        else
        {
            /* not supported, just skip it */
            bit_idx += (int)report->report_size;
            byte_idx = (bit_idx / 8);
        }

        if (byte_idx >= data_len)
        {
            break;
        }
    }

    static HID_MOUSE_FUNC *_ptr_mouse_callback;
    /* Get the mouse event callback function registered to HID class driver */
    _ptr_mouse_callback = usbh_hid_get_mouse_callback();

    if ((has_mouse_event) && (_ptr_mouse_callback != USBNULL))
    {
        _ptr_mouse_callback(hdev, &_mouse_event);
        // HID_DBGMSG("X: %d, Y: %d, W: %d, button: 0x%x\n", _mouse_event.X, _mouse_event.Y, _mouse_event.wheel, _mouse_event.button_map);
    }

    return 0;
}

/// @endcond HIDDEN_SYMBOLS
