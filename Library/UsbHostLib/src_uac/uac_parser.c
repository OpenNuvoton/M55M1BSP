/**************************************************************************//**
 * @file     uac_parser.c
 * @version  V1.00
 * @brief    USB Host audio class report descriptor parser.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "usbh_lib.h"
#include "usbh_uac.h"
#include "uac.h"

/// @cond HIDDEN_SYMBOLS

static int  uac_parse_ac_interface(UAC_DEV_T *uac, uint8_t *bptr)
{
    const AC_IT_T     *ac_itd;
    const AC_OT_T     *ac_otd;

    UAC_DBGMSG("Parse AC - [%d] [0x%x] [0x%x]\n", ((CS_HDR_T *)bptr)->bLength, ((CS_HDR_T *)bptr)->bDescriptorType, ((CS_HDR_T *)bptr)->bDescriptorSubtype);

    switch (((CS_HDR_T *)bptr)->bDescriptorSubtype)
    {
        case AC_DESCRIPTOR_UNDEFINED:       /* Not interested, discard it...              */
            UAC_DBGMSG("AC: AC_DESCRIPTOR_UNDEFINED\n");
            break;

        case HEADER:
            /* Not interested, discard it...              */
            UAC_DBGMSG("AC: HEADER\n");
            break;

        case INPUT_TERMINAL:
            ac_itd = (const AC_IT_T *)bptr;
            UAC_DBGMSG("AC: INPUT_TERMINAL\n");

            if (ac_itd->wTerminalType == UAC_TT_USB_STREAMING)
            {
                UAC_DBGMSG("USB streaming terminal found, ID=0x%x\n", ac_itd->bTerminalID);
            }
            else if ((ac_itd->wTerminalType & 0x200U) == 0x200)
            {
                UAC_DBGMSG("MICROPHONE input terminal found, ID=0x%x\n", ac_itd->bTerminalID);
                uac->acif.mic_id = ac_itd->bTerminalID;
            }
            else
            {
                UAC_DBGMSG("Unsupported INPUT TERMINAL, ignore it!\n");
            }

            UAC_DBGMSG("      bTerminalID: 0%x\n", ac_itd->bTerminalID);
            UAC_DBGMSG("      wTerminalType: 0%x\n", ac_itd->wTerminalType);
            UAC_DBGMSG("      bAssocTerminal: 0%x\n", ac_itd->bAssocTerminal);
            UAC_DBGMSG("      bNrChannels: 0%x\n", ac_itd->bNrChannels);
            UAC_DBGMSG("      wChannelConfig: 0%x\n", ac_itd->wChannelConfig);
            break;

        case OUTPUT_TERMINAL:
            ac_otd = (const AC_OT_T *)bptr;
            UAC_DBGMSG("AC: OUTPUT_TERMINAL\n");

            if (ac_otd->wTerminalType == UAC_TT_USB_STREAMING)
            {
                UAC_DBGMSG("USB streaming terminal found, ID=0x%x\n", ac_otd->bTerminalID);
            }
            else if ((ac_otd->wTerminalType & 0x300U) == 0x300)
            {
                UAC_DBGMSG("SPEAKER output terminal found, ID=0x%x\n", ac_otd->bTerminalID);
                uac->acif.speaker_id = ac_otd->bTerminalID;
                uac->acif.speaker_fuid = ac_otd->bSourceID;
            }
            else
            {
                UAC_DBGMSG("Unsupported OUTPUT TERMINAL, ignore it!\n");
            }

            UAC_DBGMSG("      bTerminalID: 0%x\n", ac_otd->bTerminalID);
            UAC_DBGMSG("      wTerminalType: 0%x\n", ac_otd->wTerminalType);
            UAC_DBGMSG("      bAssocTerminal: 0%x\n", ac_otd->bAssocTerminal);
            UAC_DBGMSG("      bSourceID: 0%x\n", ac_otd->bSourceID);
            break;

        case MIXER_UNIT:
#ifdef UAC_DEBUG
            UAC_DBGMSG("AC: MIXER_UNIT\n");
            UAC_DBGMSG("      bUnitID: 0%x\n", ((AC_MXR_T *)bptr)->bUnitID);
            UAC_DBGMSG("      bNrInPins: 0%x\n", ((AC_MXR_T *)bptr)->bNrInPins);
#endif
            break;

        case SELECTOR_UNIT:
#ifdef UAC_DEBUG
            UAC_DBGMSG("AC: SELECTOR_UNIT\n");
            UAC_DBGMSG("      bUnitID: 0%x\n", ((AC_SU_T *)bptr)->bUnitID);
            UAC_DBGMSG("      bNrInPins: 0%x\n", ((AC_SU_T *)bptr)->bNrInPins);
#endif
            break;

        case FEATURE_UNIT:

#ifdef UAC_DEBUG
            UAC_DBGMSG("AC: FEATURE_UNIT\n");
            UAC_DBGMSG("      bUnitID: 0%x\n", ((AC_FU_T *)bptr)->bUnitID);
            UAC_DBGMSG("      bSourceID: 0%x\n", ((AC_FU_T *)bptr)->bSourceID);
            UAC_DBGMSG("      bControlSize: 0%x\n", ((AC_FU_T *)bptr)->bControlSize);
#endif
            break;

        case PROCESSING_UNIT:
#ifdef UAC_DEBUG
            UAC_DBGMSG("AC: PROCESSING_UNIT\n");
            UAC_DBGMSG("      bUnitID: 0%x\n", ((AC_PU_T *)bptr)->bUnitID);
            UAC_DBGMSG("      wProcessType: 0%x\n", ((AC_PU_T *)bptr)->wProcessType);
            UAC_DBGMSG("      bNrInPins: 0%x\n", ((AC_PU_T *)bptr)->bNrInPins);
#endif
            break;

        case EXTENSION_UNIT:
            UAC_DBGMSG("AC: EXTENSION_UNIT\n");
            break;

        default:
            UAC_ERRMSG("uac_parse_ac_interface - unrecognized bDescriptorSubtype 0x%x!\n", ((CS_HDR_T *)bptr)->bDescriptorSubtype);
            return UAC_RET_PARSER;
    }

    return 0;
}

static int  uac2_parse_ac_interface(UAC_DEV_T *uac, uint8_t *bptr)
{
    AC2_IF_HDR_T  const *ac2_hdr;
    AC2_CLK_SRC_T const *clk_src;
    AC2_IT_T      const *ac2_itd;
    AC2_OT_T      const *ac2_otd;

    UAC_DBGMSG("Parse AC2 - [%d] [0x%x] [0x%x]\n", ((CS_HDR_T *)bptr)->bLength, ((CS_HDR_T *)bptr)->bDescriptorType, ((CS_HDR_T *)bptr)->bDescriptorSubtype);

    switch (((CS_HDR_T *)bptr)->bDescriptorSubtype)
    {
        case AC_DESCRIPTOR_UNDEFINED:
            UAC_DBGMSG("AC2: AC_DESCRIPTOR_UNDEFINED\n");
            break;

        case HEADER:
            ac2_hdr = (AC2_IF_HDR_T *)bptr;
            uac->acif.bcdADC = ac2_hdr->bcdADC;
            UAC_DBGMSG("AC2: HEADER (bcdADC=0x%04x)\n", ac2_hdr->bcdADC);
            UAC_DBGMSG("      bCategory: 0x%x\n", ac2_hdr->bCategory);
            UAC_DBGMSG("      wTotalLength: %d\n", ac2_hdr->wTotalLength);
            UAC_DBGMSG("      bmControls: 0x%x\n", ac2_hdr->bmControls);
            break;

        case INPUT_TERMINAL:
            ac2_itd = (AC2_IT_T *)bptr;
            UAC_DBGMSG("AC2: INPUT_TERMINAL\n");

            if (ac2_itd->wTerminalType == UAC_TT_USB_STREAMING)
            {
                UAC_DBGMSG("USB streaming terminal found, ID=0x%x\n", ac2_itd->bTerminalID);
            }
            else if ((ac2_itd->wTerminalType & 0x200) == 0x200)
            {
                UAC_DBGMSG("MICROPHONE input terminal found, ID=0x%x\n", ac2_itd->bTerminalID);
                uac->acif.mic_id = ac2_itd->bTerminalID;
            }
            else
            {
                UAC_DBGMSG("Unsupported INPUT TERMINAL, ignore it!\n");
            }

            UAC_DBGMSG("      bTerminalID: 0x%x\n", ac2_itd->bTerminalID);
            UAC_DBGMSG("      wTerminalType: 0x%x\n", ac2_itd->wTerminalType);
            UAC_DBGMSG("      bAssocTerminal: 0x%x\n", ac2_itd->bAssocTerminal);
            UAC_DBGMSG("      bCSourceID: 0x%x\n", ac2_itd->bCSourceID);
            UAC_DBGMSG("      bNrChannels: 0x%x\n", ac2_itd->bNrChannels);
            UAC_DBGMSG("      bmChannelConfig: 0x%x\n", ac2_itd->bmChannelConfig);
            break;

        case OUTPUT_TERMINAL:
            ac2_otd = (AC2_OT_T *)bptr;
            UAC_DBGMSG("AC2: OUTPUT_TERMINAL\n");

            if (ac2_otd->wTerminalType == UAC_TT_USB_STREAMING)
            {
                UAC_DBGMSG("USB streaming terminal found, ID=0x%x\n", ac2_otd->bTerminalID);
            }
            else if ((ac2_otd->wTerminalType & 0x300) == 0x300)
            {
                UAC_DBGMSG("SPEAKER output terminal found, ID=0x%x\n", ac2_otd->bTerminalID);
                uac->acif.speaker_id = ac2_otd->bTerminalID;
                uac->acif.speaker_fuid = ac2_otd->bSourceID;
            }
            else
            {
                UAC_DBGMSG("Unsupported OUTPUT TERMINAL, ignore it!\n");
            }

            UAC_DBGMSG("      bTerminalID: 0x%x\n", ac2_otd->bTerminalID);
            UAC_DBGMSG("      wTerminalType: 0x%x\n", ac2_otd->wTerminalType);
            UAC_DBGMSG("      bAssocTerminal: 0x%x\n", ac2_otd->bAssocTerminal);
            UAC_DBGMSG("      bSourceID: 0x%x\n", ac2_otd->bSourceID);
            UAC_DBGMSG("      bCSourceID: 0x%x\n", ac2_otd->bCSourceID);
            break;

        case FEATURE_UNIT:
#ifdef UAC_DEBUG
            UAC_DBGMSG("AC2: FEATURE_UNIT\n");
            UAC_DBGMSG("      bUnitID: 0x%x\n", ((AC2_FU_T *)bptr)->bUnitID);
            UAC_DBGMSG("      bSourceID: 0x%x\n", ((AC2_FU_T *)bptr)->bSourceID);
#endif
            break;

        case UAC2_CLOCK_SOURCE:
            clk_src = (AC2_CLK_SRC_T *)bptr;
            UAC_DBGMSG("AC2: CLOCK_SOURCE\n");
            UAC_DBGMSG("      bClockID: 0x%x\n", clk_src->bClockID);
            UAC_DBGMSG("      bmAttributes: 0x%x\n", clk_src->bmAttributes);
            UAC_DBGMSG("      bmControls: 0x%x\n", clk_src->bmControls);

            if (uac->acif.clk_src_id == 0U)
            {
                uac->acif.clk_src_id = clk_src->bClockID;
                UAC_DBGMSG("      Clock Source ID: 0x%02x\n", uac->acif.clk_src_id);
            }
            else if (uac->acif.mic_clk_src_id == 0U)
            {
                uac->acif.mic_clk_src_id = clk_src->bClockID;
                UAC_DBGMSG("      Microphone Clock Source ID: 0x%02x\n", uac->acif.mic_clk_src_id);
            }
            else
            {
                UAC_DBGMSG("      Clock Source ID: 0x%02x (ignored)\n", clk_src->bClockID);
            }

            break;

        case UAC2_CLOCK_SELECTOR:
#ifdef UAC_DEBUG
            UAC_DBGMSG("AC2: CLOCK_SELECTOR\n");
            UAC_DBGMSG("      bClockID: 0x%x\n", ((AC2_CLK_SEL_T *)bptr)->bClockID);
            UAC_DBGMSG("      bNrInPins: 0x%x\n", ((AC2_CLK_SEL_T *)bptr)->bNrInPins);
#endif
            break;

        case UAC2_CLOCK_MULTIPLIER:
#ifdef UAC_DEBUG
            UAC_DBGMSG("AC2: CLOCK_MULTIPLIER\n");
            UAC_DBGMSG("      bClockID: 0x%x\n", ((AC2_CLK_MUL_T *)bptr)->bClockID);
            UAC_DBGMSG("      bCSourceID: 0x%x\n", ((AC2_CLK_MUL_T *)bptr)->bCSourceID);
#endif
            break;

        case SELECTOR_UNIT:
            UAC_DBGMSG("AC2: SELECTOR_UNIT\n");
            break;

        case MIXER_UNIT:
            UAC_DBGMSG("AC2: MIXER_UNIT\n");
            break;

        case UAC2_EFFECT_UNIT:
            UAC_DBGMSG("AC2: EFFECT_UNIT\n");
            break;

        case UAC2_PROCESSING_UNIT:
            UAC_DBGMSG("AC2: PROCESSING_UNIT\n");
            break;

        case UAC2_EXTENSION_UNIT:
            UAC_DBGMSG("AC2: EXTENSION_UNIT\n");
            break;

        case UAC2_SAMPLE_RATE_CONVERTER:
            UAC_DBGMSG("AC2: SAMPLE_RATE_CONVERTER\n");
            break;

        default:
            UAC_ERRMSG("uac2_parse_ac_interface - unrecognized bDescriptorSubtype 0x%x!\n", ((CS_HDR_T *)bptr)->bDescriptorSubtype);
            return UAC_RET_PARSER;
    }

    return 0;
}

static int  uac_set_microphone_feature_unit(UAC_DEV_T *uac)
{
    const DESC_CONF_T  *config;
    const AC_FU_T      *hdr;
    uint8_t      *cfg_buff;
    uint8_t      bTerminalID = uac->acif.mic_id;
    int          parsed_len;
    int          size;

    cfg_buff = uac->udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;

    /* step over configuration descriptor */
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Find the Terminal Descriptor                                                      */
    /*------------------------------------------------------------------------------------*/
    while (size >= (int)sizeof(DESC_IF_T))
    {
        hdr = (const AC_FU_T *)&cfg_buff[parsed_len];

        if ((hdr->bDescriptorType == CS_INTERFACE) && (hdr->bDescriptorSubtype == FEATURE_UNIT) &&
                (hdr->bSourceID == bTerminalID))
        {
            uac->acif.mic_fuid = hdr->bUnitID;
            return 0;
        }

        if (hdr->bLength == 0)
        {
            return UAC_RET_PARSER;          /* prevent infinite loop                      */
        }

        parsed_len += hdr->bLength;
        size -= hdr->bLength;
    }

    return UAC_RET_PARSER;                  /* not found                                  */
}

/**
 *  @brief  Parse and get audio control (AC) interface information from descriptors.
 *  @param[in]  uac    UAC device
 *  @param[in]  iface  Audio control interface
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otherwise  Error occurred
 */
int uac_parse_control_interface(UAC_DEV_T *uac, IFACE_T *iface)
{
    const DESC_CONF_T  *config;
    const DESC_IF_T    *ifd = USBNULL;
    int          if_num;
    uint8_t      *cfg_buff;
    int          parsed_len;
    int          size;

    if_num = iface->if_num;                      /* interface number of AC interface      */

    UAC_DBGMSG("UAC parsing audio control (AC) interface %d...\n", if_num);

    (void)memset(&uac->acif, 0, sizeof(uac->acif));    /* clear AC information                  */
    uac->acif.iface = iface;

    cfg_buff = uac->udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;

    /* step over configuration descriptor */
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Find the Standard AC Interface Descriptor                                         */
    /*------------------------------------------------------------------------------------*/
    while (size >= (int)sizeof(DESC_IF_T))
    {
        ifd = (const DESC_IF_T *)&cfg_buff[parsed_len];

        if ((ifd->bDescriptorType == USB_DT_INTERFACE) && (ifd->bInterfaceNumber == if_num) &&
                (ifd->bInterfaceClass == USB_CLASS_AUDIO) && (ifd->bInterfaceSubClass == SUBCLS_AUDIOCONTROL))
        {
            break;
        }

        if (ifd->bLength == 0)
        {
            return UAC_RET_PARSER;          /* prevent infinite loop                      */
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    if (size < (int)sizeof(DESC_IF_T))           /* cannot find the Standard AC descriptor     */
    {
        UAC_ERRMSG("UAC_RET_PARSER! - AC standard not found!\n");
        return UAC_RET_PARSER;
    }

    parsed_len += ifd->bLength;
    size -= ifd->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Detect UAC version from the first CS_INTERFACE HEADER descriptor                  */
    /*------------------------------------------------------------------------------------*/
    {
        const CS_HDR_T *cs_hdr = (const CS_HDR_T *)&cfg_buff[parsed_len];

        if ((size > (int)sizeof(DESC_HDR_T)) &&
                (cs_hdr->bDescriptorType == (uint8_t)CS_INTERFACE) &&
                (cs_hdr->bDescriptorSubtype == (uint8_t)HEADER))
        {
            /* bcdADC is at offset 3 in both AC_IF_HDR_T and AC2_IF_HDR_T */
            uac->acif.bcdADC = ((AC_IF_HDR_T *)cfg_buff)->bcdADC;
            UAC_DBGMSG("UAC version detected: 0x%04x\n", uac->acif.bcdADC);
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  Walk though all Class-Specific AC Interface Descriptor (4.3.2)                    */
    /*------------------------------------------------------------------------------------*/
    while (size > (int)sizeof(DESC_HDR_T))
    {
        ifd = (const DESC_IF_T *)&cfg_buff[parsed_len];

        if (ifd->bDescriptorType != CS_INTERFACE)
        {
            break;
        }

        int          ret;

        if (uac->acif.bcdADC >= UAC_VERSION_2)
        {
            ret = uac2_parse_ac_interface(uac, &cfg_buff[parsed_len]);
        }
        else
        {
            ret = uac_parse_ac_interface(uac, &cfg_buff[parsed_len]);
        }

        if (ret < 0)
        {
            UAC_ERRMSG("UAC_RET_PARSER! - parsing CS\n");
            return UAC_RET_PARSER;
        }

        if (ifd->bLength == 0)
        {
            return UAC_RET_PARSER;          /* prevent infinite loop                      */
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    (void)uac_set_microphone_feature_unit(uac);

    UAC_DBGMSG("    Microphone Input Terminal ID: 0x%x\n", uac->acif.mic_id);
    UAC_DBGMSG("    Microphone Feature Unit ID: 0x%x\n", uac->acif.mic_fuid);
    UAC_DBGMSG("    Speaker Output Terminal ID: 0x%x\n", uac->acif.speaker_id);
    UAC_DBGMSG("    Speaker Feature Unit ID: 0x%x\n", uac->acif.speaker_fuid);

    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        UAC_DBGMSG("    UAC 2.0 Clock Source ID: 0x%x\n", uac->acif.clk_src_id);
    }

    return 0;
}

static int  uac_parse_as_interface(AS_IF_T *asif, uint8_t *bptr)
{
    const ALT_IFACE_T  *aif = asif->iface->aif;

    if (((CS_HDR_T *)bptr)->bDescriptorType == USB_DT_ENDPOINT)
    {
        int i;

        /* Find the endpoint information of this AS interface */
        for (i = 0; i < aif->ifd->bNumEndpoints; i++)
        {
            if (aif->ep[i].bEndpointAddress == ((DESC_EP_T *)bptr)->bEndpointAddress)
            {
                asif->ep = &(asif->iface->aif->ep[i]);
                break;
            }
        }

        if (i >= aif->ifd->bNumEndpoints)
        {
            UAC_ERRMSG("Endpoint 0x%x parsing error!\n", ((DESC_EP_T *)bptr)->bEndpointAddress);
            return UAC_RET_PARSER;
        }

        return 0;
    }

    if (((CS_HDR_T *)bptr)->bDescriptorType == CS_ENDPOINT)
    {
        asif->cs_epd = (CS_EP_T *)bptr;
        return 0;
    }

    switch (((CS_HDR_T *)bptr)->bDescriptorSubtype)
    {
        case AS_DESCRIPTOR_UNDEFINED:
            UAC_DBGMSG("AS: AS_DESCRIPTOR_UNDEFINED\n");
            break;

        case AS_GENERAL:
            asif->as_gen = (AS_GEN_T *)bptr;
            UAC_DBGMSG("AS: AS_GENERAL\n");
            UAC_DBGMSG("      bTerminalLink: 0%x\n", asif->as_gen->bTerminalLink);
            UAC_DBGMSG("      wFormatTag: 0%x\n", asif->as_gen->wFormatTag);
            break;

        case FORMAT_TYPE:
            asif->ft = (AS_FT1_T *)bptr;
            UAC_DBGMSG("AS: FORMAT_TYPE\n");
            break;

        case FORMAT_SPECIFIC:
            UAC_DBGMSG("AS: FORMAT_SPECIFIC\n");
            break;

        default:
            UAC_ERRMSG("uac_parse_as_interface - unrecognized bDescriptorSubtype 0x%x!\n", ((CS_HDR_T *)bptr)->bDescriptorSubtype);
            return UAC_RET_PARSER;
    }

    return 0;
}

static int  uac2_parse_as_interface(AS_IF_T *asif, uint8_t *bptr)
{
    ALT_IFACE_T  const *aif = asif->iface->aif;

    if (((CS_HDR_T *)bptr)->bDescriptorType == USB_DT_ENDPOINT)
    {
        int   i;

        for (i = 0; i < aif->ifd->bNumEndpoints; i++)
        {
            if (aif->ep[i].bEndpointAddress == ((DESC_EP_T *)bptr)->bEndpointAddress)
            {
                asif->ep = &(asif->iface->aif->ep[i]);
                break;
            }
        }

        if (i >= aif->ifd->bNumEndpoints)
        {
            UAC_ERRMSG("Endpoint 0x%x parsing error!\n", ((DESC_EP_T *)bptr)->bEndpointAddress);
            return UAC_RET_PARSER;
        }

        return 0;
    }

    if (((CS_HDR_T *)bptr)->bDescriptorType == CS_ENDPOINT)
    {
        asif->cs2_epd = (CS2_EP_T *)bptr;
        UAC_DBGMSG("AS2: CS_ENDPOINT\n");
        UAC_DBGMSG("      bmAttributes: 0x%x\n", asif->cs2_epd->bmAttributes);
        UAC_DBGMSG("      bmControls: 0x%x\n", asif->cs2_epd->bmControls);
        UAC_DBGMSG("      bLockDelayUnits: 0x%x\n", asif->cs2_epd->bLockDelayUnits);
        UAC_DBGMSG("      wLockDelay: 0x%x\n", asif->cs2_epd->wLockDelay);
        return 0;
    }

    switch (((CS_HDR_T *)bptr)->bDescriptorSubtype)
    {
        case AS_DESCRIPTOR_UNDEFINED:
            UAC_DBGMSG("AS2: AS_DESCRIPTOR_UNDEFINED\n");
            break;

        case AS_GENERAL:
            asif->as2_gen = (AS2_GEN_T *)bptr;
            UAC_DBGMSG("AS2: AS_GENERAL\n");
            UAC_DBGMSG("      bTerminalLink: 0x%x\n", asif->as2_gen->bTerminalLink);
            UAC_DBGMSG("      bmControls: 0x%x\n", asif->as2_gen->bmControls);
            UAC_DBGMSG("      bFormatType: 0x%x\n", asif->as2_gen->bFormatType);
            UAC_DBGMSG("      bmFormats: 0x%x\n", asif->as2_gen->bmFormats);
            UAC_DBGMSG("      bNrChannels: 0x%x\n", asif->as2_gen->bNrChannels);
            UAC_DBGMSG("      bmChannelConfig: 0x%x\n", asif->as2_gen->bmChannelConfig);
            break;

        case FORMAT_TYPE:
            asif->ft2 = (AS2_FT1_T *)bptr;
            UAC_DBGMSG("AS2: FORMAT_TYPE\n");
            UAC_DBGMSG("      bFormatType: 0x%x\n", asif->ft2->bFormatType);
            UAC_DBGMSG("      bSubslotSize: %d\n", asif->ft2->bSubslotSize);
            UAC_DBGMSG("      bBitResolution: %d\n", asif->ft2->bBitResolution);
            break;

        case FORMAT_SPECIFIC:
            UAC_DBGMSG("AS2: FORMAT_SPECIFIC\n");
            break;

        default:
            UAC_ERRMSG("uac2_parse_as_interface - unrecognized bDescriptorSubtype 0x%x!\n", ((CS_HDR_T *)bptr)->bDescriptorSubtype);
            return UAC_RET_PARSER;
    }

    return 0;
}

static int  iface_have_iso_in_ep(IFACE_T *iface)
{
    int         i;
    int         j;

    for (i = 0; i < (int)iface->num_alt; i++)
    {
        for (j = 0; j < iface->alt[i].ifd->bNumEndpoints; j++)
        {
            const EP_INFO_T   *ep;
            ep = &(iface->alt[i].ep[j]);

            if (ep != USBNULL)
            {
                if (((ep->bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_ISO) &&
                        ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN))
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

static int  iface_have_iso_out_ep(IFACE_T *iface)
{
    int         i;
    int         j;

    for (i = 0; i < (int)iface->num_alt; i++)
    {
        for (j = 0; j < iface->alt[i].ifd->bNumEndpoints; j++)
        {
            const EP_INFO_T   *ep;
            ep = &(iface->alt[i].ep[j]);

            if (ep != USBNULL)
            {
                if (((ep->bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_ISO) &&
                        ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT))
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

static void *uac_find_terminal(UAC_DEV_T *uac, uint8_t bTerminalID)
{
    const DESC_CONF_T  *config;
    uint8_t      *cfg_buff;
    int          parsed_len;
    int          size;

    cfg_buff = uac->udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;

    /* step over configuration descriptor */
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Find the Terminal Descriptor                                                      */
    /*------------------------------------------------------------------------------------*/
    while (size >= (int)sizeof(DESC_IF_T))
    {
        AC_OT_T *hdr = (AC_OT_T *)&cfg_buff[parsed_len];

        if ((hdr->bDescriptorType == CS_INTERFACE) &&
                ((hdr->bDescriptorSubtype == INPUT_TERMINAL) || (hdr->bDescriptorSubtype == OUTPUT_TERMINAL)) &&
                (hdr->bTerminalID == bTerminalID))
        {
            return (void *)hdr;
        }

        if (hdr->bLength == 0)
        {
            return USBNULL;                    /* prevent infinite loop                      */
        }

        parsed_len += hdr->bLength;
        size -= hdr->bLength;
    }

    return USBNULL;                            /* not found                                  */
}

#if 0
static void *uac_find_feature_unit(UAC_DEV_T *uac, uint8_t bUnitID)
{
    const DESC_CONF_T  *config;
    AC_FU_T      *hdr;
    uint8_t      *cfg_buff;
    int          parsed_len;
    int          size;

    cfg_buff = uac->udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;

    /* step over configuration descriptor */
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Find the Terminal Descriptor                                                      */
    /*------------------------------------------------------------------------------------*/
    while (size >= sizeof(DESC_IF_T))
    {
        hdr = (AC_FU_T *)&cfg_buff[parsed_len];

        if ((hdr->bDescriptorType == CS_INTERFACE) && (hdr->bDescriptorSubtype == FEATURE_UNIT) &&
                (hdr->bUnitID == bUnitID))
        {
            return (void *)hdr;
        }

        if (hdr->bLength == 0)
        {
            return USBNULL;                    /* prevent infinite loop                      */
        }

        parsed_len += hdr->bLength;
        size -= hdr->bLength;
    }

    return USBNULL;                            /* not found                                  */
}
#endif

/**
 *  @brief  Parse and get audio streaming (AS) interface information from descriptors.
 *  @param[in]  uac    UAC device
 *  @param[in]  iface  Audio control interface
 *  @param[in]  bAlternateSetting   Selected alternative interface
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otherwise  Error occurred
 */
int uac_parse_streaming_interface(UAC_DEV_T *uac, IFACE_T *iface, uint8_t bAlternateSetting)
{
    const DESC_CONF_T  *config;
    const DESC_IF_T    *ifd = USBNULL;
    AS_IF_T      asif;
    int          if_num;
    uint8_t      *cfg_buff;
    int          parsed_len;
    int          size;

    if_num = iface->if_num;                      /* interface number of AC interface      */

    UAC_DBGMSG("UAC parsing audio stream (AS) interface %d, alt %d...\n", if_num, bAlternateSetting);

    (void)memset(&asif, 0, sizeof(asif));              /* clear AS information                  */
    asif.iface = iface;

    cfg_buff = uac->udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;

    /* step over configuration descriptor */
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Find the Standard AS Interface Descriptor                                         */
    /*------------------------------------------------------------------------------------*/
    while (size >= (int)sizeof(DESC_IF_T))
    {
        ifd = (const DESC_IF_T *)&cfg_buff[parsed_len];

        if ((ifd->bDescriptorType == USB_DT_INTERFACE) &&
                (ifd->bInterfaceNumber == if_num) && (ifd->bAlternateSetting == bAlternateSetting) &&
                (ifd->bInterfaceClass == USB_CLASS_AUDIO) && (ifd->bInterfaceSubClass == SUBCLS_AUDIOSTREAMING))
        {
            break;
        }

        if (ifd->bLength == 0)
        {
            return UAC_RET_PARSER;          /* prevent infinite loop                      */
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    if (size < (int)sizeof(DESC_IF_T))           /* cannot find the Standard AC descriptor     */
    {
        UAC_ERRMSG("UAC_RET_PARSER! - AC standard not found!\n");
        return UAC_RET_PARSER;
    }

    if (ifd->bNumEndpoints == 0)
    {
        UAC_DBGMSG("This alternative interface is an idle interface.\n");
        return 0;
    }

    parsed_len += ifd->bLength;
    size -= ifd->bLength;

    /*------------------------------------------------------------------------------------*/
    /*  Walk though all Class-Specific AS Interface Descriptor (4.5.2)                    */
    /*------------------------------------------------------------------------------------*/
    while (size > (int)sizeof(DESC_HDR_T))
    {
        ifd = (const DESC_IF_T *)&cfg_buff[parsed_len];

        //UAC_DBGMSG("Parse AS - [%d] [0x%x] [0x%x]\n", ((CS_HDR_T *)bptr)->bLength, ((CS_HDR_T *)bptr)->bDescriptorType, ((CS_HDR_T *)bptr)->bDescriptorSubtype);

        if ((ifd->bDescriptorType != CS_INTERFACE) &&
                (ifd->bDescriptorType != USB_DT_ENDPOINT) &&
                (ifd->bDescriptorType != CS_ENDPOINT))
        {
            break;
        }

        int ret;

        if (uac->acif.bcdADC >= UAC_VERSION_2)
        {
            ret = uac2_parse_as_interface(&asif, &cfg_buff[parsed_len]);
        }
        else
        {
            ret = uac_parse_as_interface(&asif, &cfg_buff[parsed_len]);
        }

        if (ret < 0)
        {
            UAC_ERRMSG("UAC_RET_PARSER! - parsing CS\n");
            return UAC_RET_PARSER;
        }

        if (ifd->bLength == 0)
        {
            return UAC_RET_PARSER;          /* prevent infinite loop                      */
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        /*------------------------------------------------------------------------------------*/
        /*  UAC 2.0 path                                                                      */
        /*------------------------------------------------------------------------------------*/
        if (asif.as2_gen == USBNULL)
        {
            UAC_ERRMSG("UAC_RET_PARSER! - AS2_GEN not found!\n");
            return UAC_RET_PARSER;
        }

        if (iface_have_iso_in_ep(iface))
        {
            asif.ot2 = (AC2_OT_T *)uac_find_terminal(uac, asif.as2_gen->bTerminalLink);

            if (asif.ot2)
            {
                UAC_DBGMSG("UAC2 Audio in Terminal ID: 0x%x\n", asif.ot2->bTerminalID);
                UAC_DBGMSG("    wTerminalType: 0x%x\n", asif.ot2->wTerminalType);
                UAC_DBGMSG("    bSourceID: 0x%x\n", asif.ot2->bSourceID);
                UAC_DBGMSG("    bCSourceID: 0x%x\n", asif.ot2->bCSourceID);
            }
            else
            {
                UAC_ERRMSG("Cannot find UAC2 audio in Output Terminal %d!\n", asif.as2_gen->bTerminalLink);
            }

            (void)memcpy(&uac->asif_in, &asif, sizeof(asif));
        }
        else if (iface_have_iso_out_ep(iface))
        {
            asif.it2 = (AC2_IT_T *)uac_find_terminal(uac, asif.as2_gen->bTerminalLink);

            if (asif.it2)
            {
                UAC_DBGMSG("UAC2 Audio out Terminal ID: 0x%x\n", asif.it2->bTerminalID);
                UAC_DBGMSG("    wTerminalType: 0x%x\n", asif.it2->wTerminalType);
                UAC_DBGMSG("    bCSourceID: 0x%x\n", asif.it2->bCSourceID);
            }
            else
            {
                UAC_ERRMSG("Cannot find UAC2 audio out Input Terminal %d!\n", asif.as2_gen->bTerminalLink);
            }

            (void)memcpy(&uac->asif_out, &asif, sizeof(asif));
        }
        else
        {
            UAC_ERRMSG("Interface cannot find iso endpoints!\n");
            return UAC_RET_PARSER;
        }
    }
    else
    {
        /*------------------------------------------------------------------------------------*/
        /*  UAC 1.0 path                                                                      */
        /*------------------------------------------------------------------------------------*/
        if (asif.as_gen == USBNULL)
        {
            UAC_ERRMSG("UAC_RET_PARSER! - AS_GEN not found!\n");
            return UAC_RET_PARSER;
        }

        if (iface_have_iso_in_ep(iface))
        {
            /* Find microphone's output terminal by AS_GEN's bTerminalLink */
            asif.ot = (AC_OT_T *)uac_find_terminal(uac, asif.as_gen->bTerminalLink);

            if (asif.ot)
            {
                UAC_DBGMSG("Audio in Terminal ID: 0%x\n", asif.ot->bTerminalID);
                UAC_DBGMSG("    bDescriptorSubtype: 0%x\n", asif.ot->bDescriptorSubtype);
                UAC_DBGMSG("    wTerminalType: 0%x\n", asif.ot->wTerminalType);
                UAC_DBGMSG("    bAssocTerminal: 0%x\n", asif.ot->bAssocTerminal);
                UAC_DBGMSG("    bSourceID: 0%x\n", asif.ot->bSourceID);
            }
            else
            {
                UAC_ERRMSG("Cannot find audio in Output Terminal %d!\n", asif.as_gen->bTerminalLink);
            }

            (void)memcpy(&uac->asif_in, &asif, sizeof(asif));
        }
        else if (iface_have_iso_out_ep(iface))
        {
            asif.it = (AC_IT_T *)uac_find_terminal(uac, asif.as_gen->bTerminalLink);

            if (asif.it)
            {
                UAC_DBGMSG("Audio out Terminal ID: 0%x\n", asif.it->bTerminalID);
                UAC_DBGMSG("    bDescriptorSubtype: 0%x\n", asif.it->bDescriptorSubtype);
                UAC_DBGMSG("    wTerminalType: 0%x\n", asif.it->wTerminalType);
                UAC_DBGMSG("    bAssocTerminal: 0%x\n", asif.it->bAssocTerminal);
            }
            else
            {
                UAC_ERRMSG("Cannot find audio in Output Terminal %d!\n", asif.as_gen->bTerminalLink);
            }

            (void)memcpy(&uac->asif_out, &asif, sizeof(asif));
        }
        else
        {
            UAC_ERRMSG("Interface cannot find iso endpoints!\n");
            return UAC_RET_PARSER;
        }
    }

    UAC_DBGMSG("\n\nAudio stream interface parsing done =>\n");
    UAC_DBGMSG("    Interface: %d, Alt: %d (iface = 0x%x)\n", if_num, bAlternateSetting, asif.iface);

    if (asif.ep)
    {
        UAC_DBGMSG("    Endpoint: 0x%x, wMaxPacketSize: %d\n", asif.ep->bEndpointAddress, asif.ep->wMaxPacketSize);
    }

    UAC_DBGMSG("    as_gen: %s\n", (asif.as_gen == USBNULL) ? "Not Found" : "OK");
    UAC_DBGMSG("    it: %s\n", (asif.it == USBNULL) ? "Not Found" : "OK");
    UAC_DBGMSG("    ot: %s\n", (asif.ot == USBNULL) ? "Not Found" : "OK");
    UAC_DBGMSG("    ft: %s\n", (asif.ft == USBNULL) ? "Not Found" : "OK");
    UAC_DBGMSG("    cs_epd: %s\n", (asif.cs_epd == USBNULL) ? "Not Found" : "OK");

    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        UAC_DBGMSG("    as2_gen: %s\n", (asif.as2_gen == USBNULL) ? "Not Found" : "OK");
        UAC_DBGMSG("    it2: %s\n", (asif.it2 == USBNULL) ? "Not Found" : "OK");
        UAC_DBGMSG("    ot2: %s\n", (asif.ot2 == USBNULL) ? "Not Found" : "OK");
        UAC_DBGMSG("    ft2: %s\n", (asif.ft2 == USBNULL) ? "Not Found" : "OK");
        UAC_DBGMSG("    cs2_epd: %s\n", (asif.cs2_epd == USBNULL) ? "Not Found" : "OK");
    }

    return 0;
}

/// @endcond HIDDEN_SYMBOLS
