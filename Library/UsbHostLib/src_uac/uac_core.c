/**************************************************************************//**
 * @file     uac_core.c
 * @version  V1.00
 * @brief    USB Host library audio class core driver.
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

/** @addtogroup LIBRARY Library
  @{
*/

/** @addtogroup USBH_Library USBH Library
  @{
*/

/** @addtogroup USBH_EXPORTED_FUNCTIONS USBH Exported Functions
  @{
*/

/**
 *  @brief  Obtain Audio Class device's channel number.
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @return   Channel number or error code.
 *  @retval   < 0      Failed. UAC device may not present or function not supported.
 *  @retval   Otherwise  The channel number.
 */
int  usbh_uac_get_channel_number(UAC_DEV_T *uac, uint8_t target)
{
    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        AS2_GEN_T  const *as2_gen;

        if (target == UAC_SPEAKER)
        {
            as2_gen = uac->asif_out.as2_gen;
        }
        else
        {
            as2_gen = uac->asif_in.as2_gen;
        }

        if (!as2_gen)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        return as2_gen->bNrChannels;
    }
    else
    {
        AS_FT1_T   const *ft;

        if (target == UAC_SPEAKER)
        {
            ft = uac->asif_out.ft;
        }
        else
        {
            ft = uac->asif_in.ft;
        }

        if (!ft)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        return ft->bNrChannels;
    }
}

/**
 *  @brief  Obtain Audio Class device subframe bit resolution..
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[out] byte_cnt  The number of bytes occupied by one audio subframe. Can be 1, 2, 3 or 4.
 *  @return   Bit resolution or error code.
 *  @retval   < 0        Failed. UAC device may not present or function not supported.
 *  @retval   Otherwise  The number of effectively used bits from the available bits in an audio subframe.
 */
int  usbh_uac_get_bit_resolution(UAC_DEV_T *uac, uint8_t target, uint8_t *byte_cnt)
{
    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        AS2_FT1_T  const *ft2;

        if (target == UAC_SPEAKER)
        {
            ft2 = uac->asif_out.ft2;
        }
        else
        {
            ft2 = uac->asif_in.ft2;
        }

        if (!ft2)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        *byte_cnt = ft2->bSubslotSize;

        return ft2->bBitResolution;
    }
    else
    {
        AS_FT1_T   const *ft;

        if (target == UAC_SPEAKER)
        {
            ft = uac->asif_out.ft;
        }
        else
        {
            ft = uac->asif_in.ft;
        }

        if (!ft)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        *byte_cnt = ft->bSubframeSize;

        return ft->bBitResolution;
    }
}

/// @cond HIDDEN_SYMBOLS

static uint32_t  srate_to_u32(const uint8_t *srate)
{
    return (srate[2] << 16) | (srate[1] << 8) | srate[0];
}

/// @endcond HIDDEN_SYMBOLS

/**
 *  @brief  Get a list of sampling rate frequencies supported by the UAC device.
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[out] srate_list  A word array provided by user application to hold the sampling rate list.
 *  @param[in]  max_cnt  Available number of entries of srate_list[]. Must be > 2.
 *  @param[out] type   Indicates how the sampling frequency can be programmed.
 *                     0:  Continuous sampling frequency. srate_list[0] is the lower bound
 *                          in Hz of the sampling frequency and srate_list[1] is the upper bound.
 *                     1~255:  The number of discrete sampling frequencies supported. They are
 *                             listed in srate_list[].
 *  @return   Success or not.
 *  @retval   0        Success
 *  @retval   Otherwise  Failed
 */
int  usbh_uac_get_sampling_rate(UAC_DEV_T *uac, uint8_t target, uint32_t *srate_list,
                                int max_cnt, uint8_t *type)
{
    int        i;

    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        /*------------------------------------------------------------------------------------*/
        /*  UAC 2.0: Get sampling rates from Clock Source RANGE request                       */
        /*------------------------------------------------------------------------------------*/
        UAC2_FREQ_SUBRANGE_T  ranges[16];   /* support up to 16 discrete rates            */
        uint32_t   dMax;
        uint32_t   dMin;
        uint16_t   num_ranges;
        int        cnt;
        int        ret;

        cnt = (max_cnt < 16) ? max_cnt : 16;

        ret = usbh_uac2_clock_get_freq_range(uac, ranges, cnt, &num_ranges, target);

        if (ret < 0)
        {
            return ret;
        }

        if (num_ranges == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        dMax = ranges[0].dMAX;
        dMin = ranges[0].dMIN;

        /* Check if continuous range (single sub-range with dMIN != dMAX) */
        if ((num_ranges == 1U) && (dMin != dMax))
        {
            *type = 0;                      /* continuous */

            if (max_cnt < 2)
            {
                return UAC_RET_OUT_OF_MEMORY;
            }

            srate_list[0] = ranges[0].dMIN;
            srate_list[1] = ranges[0].dMAX;
        }
        else
        {
            /* Discrete frequencies: each sub-range has dMIN == dMAX */
            *type = (uint8_t)num_ranges;

            for (i = 0; (i < (int)num_ranges) && (i < max_cnt); i++)
            {
                srate_list[i] = ranges[i].dMIN;
            }
        }

        return 0;
    }
    else
    {
        /*------------------------------------------------------------------------------------*/
        /*  UAC 1.0: Get sampling rates from Format Type descriptor                           */
        /*------------------------------------------------------------------------------------*/
        AS_FT1_T   *ft;

        if (target == UAC_SPEAKER)
        {
            ft = uac->asif_out.ft;
        }
        else
        {
            ft = uac->asif_in.ft;
        }

        if (!ft)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        *type = ft->bSamFreqType;

        if (*type == 0)
        {
            if (max_cnt < 2)
            {
                return UAC_RET_OUT_OF_MEMORY;
            }

            srate_list[0] = srate_to_u32(&ft->tSamFreq[0][0]);
            srate_list[1] = srate_to_u32(&ft->tSamFreq[1][0]);
        }
        else
        {
            for (i = 0; i < *type; i++)
            {
                srate_list[i] = srate_to_u32(&ft->tSamFreq[i][0]);
            }
        }

        return 0;
    }
}

/**
 *  @brief  Set sampling rate frequency.
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *                     - \ref UAC_SET_MIN
 *                     - \ref UAC_GET_MIN
 *                     - \ref UAC_SET_MAX
 *                     - \ref UAC_GET_MAX
 *                     - \ref UAC_SET_RES
 *                     - \ref UAC_GET_RES
 *  @param[in]  srate  Sampling rate frequency to be set or get.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otherwise  Error occurred
 */
int  usbh_uac_sampling_rate_control(UAC_DEV_T *uac, uint8_t target, uint8_t req, uint32_t *srate)
{
    uint32_t    xfer_len;

    /*------------------------------------------------------------------------------------*/
    /*  UAC 2.0: redirect to Clock Source CUR request                                     */
    /*------------------------------------------------------------------------------------*/
    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        if (req == (uint8_t)UAC_SET_CUR)
        {
            return usbh_uac2_clock_set_freq(uac, target, *srate);
        }
        else if (req == (uint8_t)UAC_GET_CUR)
        {
            return usbh_uac2_clock_get_freq(uac, target, srate);
        }
        else
        {
            return UAC_RET_DRV_NOT_SUPPORTED;   /* UAC2 only supports CUR/RANGE       */
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  UAC 1.0: Endpoint sampling frequency control                                      */
    /*------------------------------------------------------------------------------------*/
    {
        EP_INFO_T   const *ep;
        uint8_t     bmRequestType;
        uint8_t     tSampleFreq[3];

        if (target == UAC_SPEAKER)
        {
            ep = uac->asif_out.ep;
        }
        else
        {
            ep = uac->asif_in.ep;
        }

        if (ep == USBNULL)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }

        tSampleFreq[0] = *srate & 0xffU;
        tSampleFreq[1] = (*srate >> 8) & 0xffU;
        tSampleFreq[2] = (*srate >> 16) & 0xffU;

        if (req & 0x80U)
        {
            uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_EP;
            bmRequestType = (uint8_t)u32ReqRequestType_tmp;
        }
        else
        {
            uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_OUT | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_EP;
            bmRequestType = (uint8_t)u32ReqRequestType_tmp;
        }

        int         ret;
        /* Audio Class Request - Endpoint Control Requests (5.2.3.2) */
        ret = usbh_ctrl_xfer(uac->udev, bmRequestType, req,
                             ((uint32_t)SAMPLING_FREQ_CONTROL << 8),  /* wValue - Control Selector (CS) */
                             ep->bEndpointAddress,         /* wIndex - endpoint              */
                             3,                            /* wLength - parameter block length */
                             tSampleFreq,                  /* parameter block                */
                             &xfer_len, UAC_REQ_TIMEOUT);

        if (ret < 0)
        {
            return ret;
        }

        if (xfer_len != 3U)
        {
            return UAC_RET_DATA_LEN;
        }

        *srate = srate_to_u32(tSampleFreq);
        return 0;
    }
}

/**
 *  @brief  Control Audio Class device mute on/off.
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *  @param[in]  chn    The requested channel. It can be one of the followings:
 *                     - \ref UAC_CH_LEFT_FRONT
 *                     - \ref UAC_CH_RIGHT_FRONT
 *                     - \ref UAC_CH_CENTER_FRONT
 *                     - \ref UAC_CH_LOW_FREQ_EN
 *                     - \ref UAC_CH_LEFT_SRN
 *                     - \ref UAC_CH_RIGHT_SRN
 *                     - \ref UAC_CH_LEFT_OF_CENTER
 *                     - \ref UAC_CH_RIGHT_OF_CENTER
 *                     - \ref UAC_CH_SURROUND
 *                     - \ref UAC_CH_SIDE_LEFT
 *                     - \ref UAC_CH_SIDE_RIGHT
 *                     - \ref UAC_CH_TOP
 *  @param[in]  mute   One byte data. If the channel is muted, then the value is 1. Otherwise, it's 0.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   UAC_RET_DEV_NOT_SUPPORTED  This UAC device does not support this function.
 *  @retval   Otherwise  Error occurred
 */
int usbh_uac_mute_control(UAC_DEV_T *uac, uint8_t target, uint8_t req, uint16_t chn, uint8_t *mute)
{
    uint8_t     bmRequestType;
    uint8_t     bUnitID;
    uint32_t    xfer_len;
    int         ret;

    if (target == UAC_MICROPHONE)
    {
        bUnitID = uac->acif.mic_fuid;
    }
    else
    {
        bUnitID = uac->acif.speaker_fuid;
    }

    /*------------------------------------------------------------------------------------*/
    /*  UAC 2.0: Use CUR request code (direction encoded in bmRequestType only)           */
    /*------------------------------------------------------------------------------------*/
    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        if ((req != (uint8_t)UAC_SET_CUR) && (req != (uint8_t)UAC_GET_CUR))
        {
            return UAC_RET_DRV_NOT_SUPPORTED;
        }

        if (req == (uint8_t)UAC_GET_CUR)
        {
            uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
            bmRequestType = (uint8_t)u32ReqRequestType_tmp;
        }
        else
        {
            uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_OUT | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
            bmRequestType = (uint8_t)u32ReqRequestType_tmp;
        }

        /* UAC2 Feature Unit Control Request (5.2.5.7, UAC2) */
        ret = usbh_ctrl_xfer(uac->udev, bmRequestType, UAC2_CUR,
                             (uint16_t)((((uint32_t)UAC2_FU_MUTE_CONTROL << 8) | (uint32_t)chn)),
                             (uint16_t)(((uint32_t)bUnitID << 8U) | (uint32_t)(uac->acif.iface->if_num)),
                             1, mute, &xfer_len, UAC_REQ_TIMEOUT);

        if (ret < 0)
        {
            return ret;
        }

        if (xfer_len != 1U)
        {
            return UAC_RET_DATA_LEN;
        }

        return 0;
    }

    /*------------------------------------------------------------------------------------*/
    /*  UAC 1.0: Original request path                                                    */
    /*------------------------------------------------------------------------------------*/
    if (req & 0x80U)
    {
        uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
        bmRequestType = (uint8_t)u32ReqRequestType_tmp;
    }
    else
    {
        uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_OUT | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
        bmRequestType = (uint8_t)u32ReqRequestType_tmp;
    }

    /* Audio Class Request - Feature Unit Control Request (5.2.2.4) */
    ret = usbh_ctrl_xfer(uac->udev, bmRequestType, req,
                         (uint16_t)(((uint32_t)MUTE_CONTROL << 8U) | (uint32_t)chn),  /* wValue - Control Selector (CS)    */
                         (uint16_t)(((uint32_t)bUnitID << 8U) | (uint32_t)(uac->acif.iface->if_num)),  /* wIndex - unit ID and interface number */
                         1,                          /* wLength - parameter block length  */
                         mute,                       /* parameter block                   */
                         &xfer_len, UAC_REQ_TIMEOUT);

    if (ret < 0)
    {
        return ret;
    }

    if (xfer_len != 1U)
    {
        return UAC_RET_DATA_LEN;
    }

    return 0;
}

/**
 *  @brief  Audio Class device volume control.
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *                     - \ref UAC_SET_MIN
 *                     - \ref UAC_GET_MIN
 *                     - \ref UAC_SET_MAX
 *                     - \ref UAC_GET_MAX
 *                     - \ref UAC_SET_RES
 *                     - \ref UAC_GET_RES
 *                     - \ref UAC_GET_STAT
 *  @param[in]  chn    The requested channel. It can be one of the followings:
 *                     - \ref UAC_CH_LEFT_FRONT
 *                     - \ref UAC_CH_RIGHT_FRONT
 *                     - \ref UAC_CH_CENTER_FRONT
 *                     - \ref UAC_CH_LOW_FREQ_EN
 *                     - \ref UAC_CH_LEFT_SRN
 *                     - \ref UAC_CH_RIGHT_SRN
 *                     - \ref UAC_CH_LEFT_OF_CENTER
 *                     - \ref UAC_CH_RIGHT_OF_CENTER
 *                     - \ref UAC_CH_SURROUND
 *                     - \ref UAC_CH_SIDE_LEFT
 *                     - \ref UAC_CH_SIDE_RIGHT
 *                     - \ref UAC_CH_TOP
 *  @param[in]  volume   Audio Class device volume value, which is interpreted as the following:
 *                       0x7FFF:    127.9961 dB
 *                       . . .
 *                       0x0100:      1.0000 dB
 *                       . . .
 *                       0x0002:      0.0078 dB
 *                       0x0001:      0.0039 dB
 *                       0x0000:      0.0000 dB
 *                       0xFFFF:     -0.0039 dB
 *                       0xFFFE:     -0.0078 dB
 *                       . . .
 *                       0xFE00:     -1.0000 dB
 *                       . . .
 *                       0x8002:   -127.9922 dB
 *                       0x8001:   -127.9961 dB
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   UAC_RET_DEV_NOT_SUPPORTED  This UAC device does not support this function.
 *  @retval   Otherwise  Error occurred
 */
int usbh_uac_vol_control(UAC_DEV_T *uac, uint8_t target, uint8_t req, uint16_t chn, uint16_t *volume)
{
    uint8_t     bmRequestType;
    uint8_t     bUnitID;
    uint32_t    xfer_len;
    int         ret;

    if (target == UAC_MICROPHONE)
    {
        bUnitID = uac->acif.mic_fuid;
    }
    else
    {
        bUnitID = uac->acif.speaker_fuid;
    }

    /*------------------------------------------------------------------------------------*/
    /*  UAC 2.0: CUR for GET/SET current; RANGE for GET MIN/MAX/RES                       */
    /*------------------------------------------------------------------------------------*/
    if (uac->acif.bcdADC >= UAC_VERSION_2)
    {
        if ((req == (uint8_t)UAC_SET_CUR) || (req == (uint8_t)UAC_GET_CUR))
        {
            if (req == (uint8_t)UAC_GET_CUR)
            {
                uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
                bmRequestType = (uint8_t)u32ReqRequestType_tmp;
            }
            else
            {
                uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_OUT | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
                bmRequestType = (uint8_t)u32ReqRequestType_tmp;
            }

            /* UAC2 Feature Unit CUR request (5.2.5.7.1, UAC2) */
            ret = usbh_ctrl_xfer(uac->udev, bmRequestType, UAC2_CUR,
                                 (uint16_t)(((uint32_t)VOLUME_CONTROL << 8U) | (uint32_t)chn),
                                 (uint16_t)(((uint32_t)bUnitID << 8U) | (uint32_t)(uac->acif.iface->if_num)),
                                 2, (uint8_t *)volume, &xfer_len, UAC_REQ_TIMEOUT);

            if (ret < 0)
            {
                return ret;
            }

            if (xfer_len != 2U)
            {
                return UAC_RET_DATA_LEN;
            }

            return 0;
        }
        else if ((req == (uint8_t)UAC_GET_MIN) || (req == (uint8_t)UAC_GET_MAX) || (req == (uint8_t)UAC_GET_RES))
        {
            /* UAC2 Feature Unit RANGE request (5.2.5.7.2, UAC2)
             * Response: wNumSubRanges(2) + N * { wMIN(2), wMAX(2), wRES(2) }
             * We read the first sub-range and extract the requested field.
             */
            uint8_t  range_buf[8];          /* 2 + 1 * 6 = 8 bytes for first sub-range   */
            uint32_t  u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
            bmRequestType = (uint8_t)u32ReqRequestType_tmp;

            ret = usbh_ctrl_xfer(uac->udev, bmRequestType, UAC2_RANGE,
                                 (uint16_t)(((uint32_t)UAC2_FU_VOLUME_CONTROL << 8U) | (uint32_t)chn),
                                 (uint16_t)(((uint32_t)bUnitID << 8U) | (uint32_t)(uac->acif.iface->if_num)),
                                 8, range_buf, &xfer_len, UAC_REQ_TIMEOUT);

            if (ret < 0)
            {
                return ret;
            }

            if (xfer_len < 8U)
            {
                return UAC_RET_DATA_LEN;
            }

            /* Extract from first sub-range */
            if (req == (uint8_t)UAC_GET_MIN)
            {
                *volume = (uint16_t)(range_buf[2] | (range_buf[3] << 8));
            }
            else if (req == (uint8_t)UAC_GET_MAX)
            {
                *volume = (uint16_t)(range_buf[4] | (range_buf[5] << 8));
            }
            else /* UAC_GET_RES */
            {
                *volume = (uint16_t)(range_buf[6] | (range_buf[7] << 8));
            }

            return 0;
        }
        else
        {
            return UAC_RET_DRV_NOT_SUPPORTED;
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  UAC 1.0: Original request path                                                    */
    /*------------------------------------------------------------------------------------*/
    if (req & 0x80U)
    {
        uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
        bmRequestType = (uint8_t)u32ReqRequestType_tmp;
    }
    else
    {
        uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_OUT | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
        bmRequestType = (uint8_t)u32ReqRequestType_tmp;
    }

    /* Audio Class Request - Feature Unit Control Request (5.2.2.4) */
    ret = usbh_ctrl_xfer(uac->udev, bmRequestType, req,
                         (uint16_t)(((uint32_t)VOLUME_CONTROL << 8U) | (uint32_t)chn),/* wValue - Control Selector (CS)    */
                         (uint16_t)(((uint32_t)bUnitID << 8U) | (uint32_t)(uac->acif.iface->if_num)),  /* wIndex - unit ID and interface number */
                         2,                          /* wLength - parameter block length  */
                         (uint8_t *)volume,          /* parameter block                   */
                         &xfer_len, UAC_REQ_TIMEOUT);

    if (ret < 0)
    {
        return ret;
    }

    if (xfer_len != 2U)
    {
        return UAC_RET_DATA_LEN;
    }

    return 0;
}

/**
 *  @brief  Audio Class device automatic gain control.
 *  @param[in]  uac  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *  @param[in]  chn    The requested channel. It can be one of the followings:
 *                     - \ref UAC_CH_LEFT_FRONT
 *                     - \ref UAC_CH_RIGHT_FRONT
 *                     - \ref UAC_CH_CENTER_FRONT
 *                     - \ref UAC_CH_LOW_FREQ_EN
 *                     - \ref UAC_CH_LEFT_SRN
 *                     - \ref UAC_CH_RIGHT_SRN
 *                     - \ref UAC_CH_LEFT_OF_CENTER
 *                     - \ref UAC_CH_RIGHT_OF_CENTER
 *                     - \ref UAC_CH_SURROUND
 *                     - \ref UAC_CH_SIDE_LEFT
 *                     - \ref UAC_CH_SIDE_RIGHT
 *                     - \ref UAC_CH_TOP
 *  @param[in]  bAGC   One byte data. If the channel's automatic gain control is on, then the value is 1. Otherwise, it's 0.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   UAC_RET_DEV_NOT_SUPPORTED  This UAC device does not support this function.
 *  @retval   Otherwise  Error occurred
 */
int  usbh_uac_auto_gain_control(UAC_DEV_T *uac, uint8_t target, uint8_t req, uint16_t chn, uint8_t *bAGC)
{
    uint8_t     bmRequestType;
    uint8_t     bUnitID;
    uint32_t    xfer_len;
    int         ret;

    if (target == UAC_MICROPHONE)
    {
        bUnitID = uac->acif.mic_fuid;
    }
    else
    {
        bUnitID = uac->acif.speaker_fuid;
    }

    if (req & 0x80U)
    {
        uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_IN | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
        bmRequestType = (uint8_t)u32ReqRequestType_tmp;
    }
    else
    {
        uint32_t u32ReqRequestType_tmp = (uint32_t)REQ_TYPE_OUT | (uint32_t)REQ_TYPE_CLASS_DEV | (uint32_t)REQ_TYPE_TO_IFACE;
        bmRequestType = (uint8_t)u32ReqRequestType_tmp;
    }

    /* Audio Class Request - Feature Unit Control Request (5.2.2.4) */
    ret = usbh_ctrl_xfer(uac->udev, bmRequestType, req,
                         ((uint16_t)((uint32_t)AUTOMATIC_GAIN_CONTROL << 8U) | (uint16_t)chn),  /* wValue - Control Selector (CS)    */
                         ((uint16_t)((uint32_t)bUnitID << 8U) | (uint16_t)(uac->acif.iface->if_num)),  /* wIndex - unit ID and interface number */
                         1,                          /* wLength - parameter block length  */
                         bAGC,                       /* parameter block                   */
                         &xfer_len, UAC_REQ_TIMEOUT);

    if (ret < 0)
    {
        return ret;
    }

    if (xfer_len != 1U)
    {
        return UAC_RET_DATA_LEN;
    }

    return 0;
}

/// @cond HIDDEN_SYMBOLS

/*====================================================================================*/
/*  UAC 2.0 Clock Source Control Functions                                            */
/*====================================================================================*/

/**
 *  @brief  Get UAC 2.0 Clock Source ID for a given target.
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target (UAC_SPEAKER or UAC_MICROPHONE)
 *  @return   Clock Source ID or error code.
 *  @retval   > 0      Clock Source ID
 *  @retval   < 0      Error
 */
int usbh_uac2_get_clock_source_id(UAC_DEV_T *uac, uint8_t target)
{
    if (uac->acif.bcdADC < UAC_VERSION_2)
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    if (uac->acif.clk_src_id == 0U)
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    if (uac->acif.mic_clk_src_id == 0U)
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    if (target == UAC_SPEAKER)
    {
        return (int)uac->acif.clk_src_id;
    }
    else if (target == UAC_MICROPHONE)
    {
        return (int)uac->acif.mic_clk_src_id;
    }
    else
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }
}

/**
 *  @brief  UAC 2.0 - Set Clock Source sampling frequency (CUR request).
 *  @param[in]  uac    UAC device
 *  @param[in]  freq   Sampling frequency in Hz
 *  @param[in]  target Select the control target (UAC_SPEAKER or UAC_MICROPHONE)
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otherwise  Error occurred
 */
int usbh_uac2_clock_set_freq(UAC_DEV_T *uac, uint8_t target, uint32_t freq)
{
    uint8_t     data[4];
    uint32_t    xfer_len;
    int         ret;

    if (target == UAC_SPEAKER)
    {
        if (uac->acif.clk_src_id == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }
    }
    else if (target == UAC_MICROPHONE)
    {
        if (uac->acif.mic_clk_src_id == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }
    }
    else
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    data[0] = freq & 0xFFU;
    data[1] = (freq >> 8) & 0xFFU;
    data[2] = (freq >> 16) & 0xFFU;
    data[3] = (freq >> 24) & 0xFFU;

    /* UAC2 Clock Source SET CUR (5.2.5.1, UAC2) */
    ret = usbh_ctrl_xfer(uac->udev,
                         REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE,  /* 0x21 */
                         UAC2_CUR,                                               /* bRequest */
                         ((uint16_t)((uint32_t)UAC2_CS_SAM_FREQ_CONTROL << 8U)),                        /* wValue */
                         (uint16_t)((((uint16_t)((target == UAC_SPEAKER) ? uac->acif.clk_src_id : uac->acif.mic_clk_src_id)) << 8U) | (uint16_t)(uac->acif.iface->if_num)), /* wIndex */
                         4,                                                      /* wLength */
                         data,
                         &xfer_len, UAC_REQ_TIMEOUT);

    if (ret < 0)
    {
        return ret;
    }

    if (xfer_len != 4U)
    {
        return UAC_RET_DATA_LEN;
    }

    return 0;
}

/**
 *  @brief  UAC 2.0 - Get Clock Source current sampling frequency (CUR request).
 *  @param[in]  uac    UAC device
 *  @param[in]  target Select the control target (UAC_SPEAKER or UAC_MICROPHONE)
 *  @param[out] freq   Current sampling frequency in Hz
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otherwise  Error occurred
 */
int usbh_uac2_clock_get_freq(UAC_DEV_T *uac, uint8_t target, uint32_t *freq)
{
    uint8_t     data[4];
    uint32_t    xfer_len;
    int         ret;

    if (target == UAC_SPEAKER)
    {
        if (uac->acif.clk_src_id == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }
    }
    else if (target == UAC_MICROPHONE)
    {
        if (uac->acif.mic_clk_src_id == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }
    }
    else
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    (void)memset(data, 0, sizeof(data));

    /* UAC2 Clock Source GET CUR (5.2.5.1, UAC2) */
    ret = usbh_ctrl_xfer(uac->udev,
                         REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE,   /* 0xA1 */
                         UAC2_CUR,                                               /* bRequest */
                         ((uint16_t)((uint32_t)UAC2_CS_SAM_FREQ_CONTROL << 8U)),                        /* wValue */
                         (uint16_t)((((uint16_t)((target == UAC_SPEAKER) ? uac->acif.clk_src_id : uac->acif.mic_clk_src_id)) << 8U) | (uint16_t)(uac->acif.iface->if_num)), /* wIndex */
                         4,                                                      /* wLength */
                         data,
                         &xfer_len, UAC_REQ_TIMEOUT);

    if (ret < 0)
    {
        return ret;
    }

    if (xfer_len != 4U)
    {
        return UAC_RET_DATA_LEN;
    }

    *freq = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    return 0;
}

/**
 *  @brief  UAC 2.0 - Get Clock Source supported frequency range (RANGE request).
 *  @param[in]  uac        UAC device
 *  @param[out] ranges     Array of frequency sub-ranges (dMIN, dMAX, dRES)
 *  @param[in]  max_cnt    Maximum number of sub-ranges the caller can accept
 *  @param[out] num_ranges Actual number of sub-ranges returned by the device
 *  @param[in]  target     Select the control target (UAC_SPEAKER or UAC_MICROPHONE)
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otherwise  Error occurred
 */
int usbh_uac2_clock_get_freq_range(UAC_DEV_T *uac, UAC2_FREQ_SUBRANGE_T *ranges,
                                   int max_cnt, uint16_t *num_ranges, uint8_t target)
{
    uint8_t     buff[(2 + (16 * 12))];         /* 2 bytes header + up to 16 sub-ranges       */
    uint32_t    xfer_len;
    uint16_t    wlen;
    int         cnt;
    int         i;
    int         ret;

    if (target == UAC_SPEAKER)
    {
        if (uac->acif.clk_src_id == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }
    }
    else if (target == UAC_MICROPHONE)
    {
        if (uac->acif.mic_clk_src_id == 0U)
        {
            return UAC_RET_DEV_NOT_SUPPORTED;
        }
    }
    else
    {
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    /* Calculate request length: 2 (wNumSubRanges) + max_cnt * 12 (each sub-range) */
    cnt = (max_cnt > 16) ? 16 : max_cnt;
    uint32_t u32Wlen_tmp = 2UL + (((uint32_t)cnt) * 12UL);
    wlen = (uint16_t)u32Wlen_tmp;

    if (wlen > sizeof(buff))
    {
        wlen = sizeof(buff);
    }

    (void)memset(buff, 0, wlen);

    /* UAC2 Clock Source GET RANGE (5.2.5.2, UAC2) */
    ret = usbh_ctrl_xfer(uac->udev,
                         REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE,   /* 0xA1 */
                         UAC2_RANGE,                                             /* bRequest */
                         (uint16_t)((uint32_t)UAC2_CS_SAM_FREQ_CONTROL << 8U),                        /* wValue */
                         (uint16_t)((((uint16_t)((target == UAC_SPEAKER) ? uac->acif.clk_src_id : uac->acif.mic_clk_src_id)) << 8U) | (uint16_t)(uac->acif.iface->if_num)), /* wIndex */
                         wlen,                                                   /* wLength */
                         buff,
                         &xfer_len, UAC_REQ_TIMEOUT);

    if (ret < 0)
    {
        return ret;
    }

    if (xfer_len < 2U)
    {
        return UAC_RET_DATA_LEN;
    }

    *num_ranges = buff[0] | (buff[1] << 8);

    /* Limit to what we can store */
    cnt = (*num_ranges > max_cnt) ? max_cnt : *num_ranges;

    /* Verify we received enough data */
    uint32_t u32Rx_xfer_len = 2UL + (((uint32_t)cnt) * 12UL);

    if (xfer_len < u32Rx_xfer_len)
    {
        cnt = (xfer_len - 2U) / 12U;
    }

    for (i = 0; i < cnt; i++)
    {
        uint32_t u32Offset_tmp = 2UL + (((uint32_t)i) * 12UL);
        uint8_t const *p = &buff[u32Offset_tmp];
        ranges[i].dMIN = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
        ranges[i].dMAX = p[4] | (p[5] << 8) | (p[6] << 16) | (p[7] << 24);
        ranges[i].dRES = p[8] | (p[9] << 8) | (p[10] << 16) | (p[11] << 24);
    }

    UAC_DBGMSG("UAC2 Clock RANGE: %d sub-ranges\n", *num_ranges);

    for (i = 0; i < cnt; i++)
    {
        UAC_DBGMSG("  [%d] dMIN=%d, dMAX=%d, dRES=%d\n", i,
                   ranges[i].dMIN, ranges[i].dMAX, ranges[i].dRES);
    }

    return 0;
}

/**
 *  @brief    Find the alternative interface with the highest PCM bit resolution (UAC 2.0).
 *            When multiple alt settings share the same wMaxPacketSize (common in UAC 2.0),
 *            this function selects the one with the highest bBitResolution among PCM formats.
 *  @param[in]  uac       UAC device (needed to access raw descriptor buffer)
 *  @param[in]  iface     USB device interface
 *  @param[in]  dir       Endpoint bEndpointAddress[7] direction
 *  @param[in]  attr      Endpoint bmAttributes[1:0] transfer type
 *  @param[in]  max_bitres  Maximum bit resolution to consider (0 = no limit).
 *                          Use this to match the peer interface's bit depth, e.g.
 *                          pass the mic's bit resolution when selecting speaker alt
 *                          to avoid unnecessary format conversion in the application.
 *  @param[out] bAlternateSetting   The alternative interface number if found.
 *  @return   Success or not.
 *  @retval     0         Success.
 *  @retval    Otherwise  Failed
 */
int  usbh_uac2_find_pcm_alt(UAC_DEV_T *uac, IFACE_T *iface, uint8_t dir, uint8_t attr, uint8_t max_bitres, uint8_t *bAlternateSetting)
{
    const DESC_CONF_T  *config;
    const DESC_IF_T    *ifd;
    uint8_t      *cfg_buff;
    int          parsed_len;
    int          size;
    int          if_num = iface->if_num;
    uint8_t      best_alt = 0;
    uint8_t      best_bitres = 0;
    uint8_t      found = 0;

    /* Walk through the raw configuration descriptor to find AS descriptors per alt */
    cfg_buff = uac->udev->cfd_buff;
    config = (const DESC_CONF_T *)cfg_buff;
    parsed_len = config->bLength;
    size = config->wTotalLength - config->bLength;

    while (size >= (int)sizeof(DESC_IF_T))
    {
        ifd = (const DESC_IF_T *)&cfg_buff[parsed_len];

        if (ifd->bLength == 0U)
        {
            break;
        }

        /* Look for AS interface descriptors matching our interface number */
        if ((ifd->bDescriptorType == USB_DT_INTERFACE) &&
                (ifd->bInterfaceNumber == if_num) &&
                (ifd->bInterfaceClass == USB_CLASS_AUDIO) &&
                (ifd->bInterfaceSubClass == SUBCLS_AUDIOSTREAMING) &&
                (ifd->bNumEndpoints > 0U))
        {
            uint8_t  cur_alt = ifd->bAlternateSetting;
            uint8_t  has_iso_ep = 0;
            uint8_t  is_pcm = 0;
            uint8_t  cur_bitres = 0;
            uint8_t  stop_scan = 0;
            int      inner_parsed_len;
            int      inner_size;

            /* Verify this alt has the right ISO endpoint direction */
            {
                uint8_t j;

                for (j = 0; j < iface->num_alt; j++)
                {
                    DESC_IF_T const *alt_ifd = iface->alt[j].ifd;

                    if (alt_ifd == USBNULL)
                    {
                        continue;
                    }

                    if (alt_ifd->bAlternateSetting == cur_alt)
                    {
                        uint8_t k;

                        for (k = 0; k < alt_ifd->bNumEndpoints; k++)
                        {
                            EP_INFO_T const *ep = &iface->alt[j].ep[k];

                            if (((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == dir) &&
                                    ((ep->bmAttributes & EP_ATTR_TT_MASK) == attr))
                            {
                                has_iso_ep = 1;
                                break;
                            }
                        }

                        break;
                    }
                }
            }

            if (!has_iso_ep)
            {
                parsed_len += ifd->bLength;
                size -= ifd->bLength;
                continue;
            }

            /* Scan the CS descriptors following this interface descriptor */
            inner_parsed_len = parsed_len + ifd->bLength;
            inner_size = size - ifd->bLength;

            while ((inner_size > (int)sizeof(DESC_HDR_T)) && (stop_scan == 0U))
            {
                const CS_HDR_T *cs;

                cs = (const CS_HDR_T *)&cfg_buff[inner_parsed_len];

                if ((cs->bLength == 0U) || (cs->bDescriptorType == USB_DT_INTERFACE))
                {
                    stop_scan = 1U;
                }
                else if (cs->bDescriptorType == CS_INTERFACE)
                {
                    if (cs->bDescriptorSubtype == AS_GENERAL)
                    {
                        if (cs->bLength < sizeof(AS2_GEN_T))
                        {
                            stop_scan = 1U;
                        }
                        else
                        {
                            const AS2_GEN_T *gen;
                            gen = (const AS2_GEN_T *)&cfg_buff[inner_parsed_len];

                            /* bmFormats bit 0 = PCM.  Reject if only RAW (bit 31). */
                            if ((gen->bmFormats & 0x01U) != 0U)
                            {
                                is_pcm = 1;
                            }
                        }
                    }
                    else if (cs->bDescriptorSubtype == FORMAT_TYPE)
                    {
                        const AS2_FT1_T *ft;

                        if (cs->bLength < sizeof(AS2_FT1_T))
                        {
                            stop_scan = 1U;
                        }
                        else
                        {
                            ft = (const AS2_FT1_T *)&cfg_buff[inner_parsed_len];
                            cur_bitres = ft->bBitResolution;
                        }
                    }
                    else
                    {
                        /* Ignore other CS descriptors */
                    }
                }
                else
                {
                    /* Ignore other descriptors */
                }

                if (stop_scan == 0U)
                {
                    inner_parsed_len += cs->bLength;
                    inner_size -= cs->bLength;
                }
            }

            if (is_pcm && (cur_bitres > best_bitres) &&
                    ((max_bitres == 0U) || (cur_bitres <= max_bitres)))
            {
                best_bitres = cur_bitres;
                best_alt = cur_alt;
                found = 1;
            }
        }

        parsed_len += ifd->bLength;
        size -= ifd->bLength;
    }

    if (!found)
    {
        return USBH_ERR_NOT_FOUND;
    }

    *bAlternateSetting = best_alt;
    UAC_DBGMSG("UAC2 find_pcm_alt: iface %d => alt %d (%d-bit PCM, limit %d)\n", if_num, best_alt, best_bitres, max_bitres);
    return 0;
}

/**
 *  @brief    Find the alternative interface whose endpoint has the maximum packet size.
 *  @param[in]  ifcae     USB device interface
 *  @param[in]  dir       Endpoint bEndpointAddress[7] direction
 *  @param[in]  attr      Endpoint bmAttributes[1:0] transfer type
 *  @param[out] bAlternateSetting   The alternative interface number if found.
 *  @return   Success or not.
 *  @retval     0         Success.
 *  @retval    Otherwise  Failed
 */
int  usbh_uac_find_max_alt(IFACE_T *iface, uint8_t dir, uint8_t attr, uint8_t *bAlternateSetting)
{
    const EP_INFO_T    *ep;
    uint8_t      i;
    uint8_t      j;
    uint16_t     wMaxPacketSize = 0;

    for (i = 0; i < iface->num_alt; i++)
    {
        for (j = 0; j < iface->alt[i].ifd->bNumEndpoints; j++)
        {
            ep = &(iface->alt[i].ep[j]);    /* get endpoint                               */

            if (((ep->bEndpointAddress & EP_ADDR_DIR_MASK) != dir) ||
                    ((ep->bmAttributes & EP_ATTR_TT_MASK) != attr))
            {
                continue;                   /* not interested endpoint                    */
            }

            if (ep->wMaxPacketSize > wMaxPacketSize)
            {
                /* a better candidate endpoint found          */
                *bAlternateSetting = i;
                wMaxPacketSize = ep->wMaxPacketSize;
            }
        }
    }

    if (wMaxPacketSize == 0U)
    {
        return USBH_ERR_NOT_FOUND;
    }

    return 0;
}

/**
 *  @brief    Find the alternative interface whose endpoint is the best choice.
 *  @param[in]  ifcae     USB device interface
 *  @param[in]  dir       Endpoint bEndpointAddress[7] direction
 *  @param[in]  attr      Endpoint bmAttributes[1:0] transfer type
 *  @param[in]  pkt_sz    Find the endpoint whose wMaxPacketSize is larger than <pkt_sz> and is most cross to <pkt_sz>.
 *  @param[out] bAlternateSetting   The alternative interface number if found.
 *  @return   Success or not.
 *  @retval     0         Success.
 *  @retval    Otherwise  Failed
 */
int  usbh_uac_find_best_alt(IFACE_T *iface, uint8_t dir, uint8_t attr, int pkt_sz, uint8_t *bAlternateSetting)
{
    const EP_INFO_T    *ep;
    uint8_t      i;
    uint8_t      j;
    uint16_t     wMaxPacketSize = 0xFFFFU;

    for (i = 0; i < iface->num_alt; i++)
    {
        for (j = 0; j < iface->alt[i].ifd->bNumEndpoints; j++)
        {
            ep = &(iface->alt[i].ep[j]);    /* get endpoint                               */

            if (((ep->bEndpointAddress & EP_ADDR_DIR_MASK) != dir) ||
                    ((ep->bmAttributes & EP_ATTR_TT_MASK) != attr))
            {
                continue;                   /* not interested endpoint                    */
            }

            if ((ep->wMaxPacketSize >= (uint16_t)pkt_sz) && (ep->wMaxPacketSize < wMaxPacketSize))
            {
                /* a better candidate endpoint found          */
                *bAlternateSetting = i;
                wMaxPacketSize = ep->wMaxPacketSize;
            }
        }
    }

    if (wMaxPacketSize == 0xFFFFU)
    {
        UAC_DBGMSG("Audio interface %d cannot find endpoint with wMaxPacketSize >= %d!\n", iface->if_num, pkt_sz);
        return USBH_ERR_NOT_FOUND;
    }

    return 0;
}

static void uac_iso_in_irq(UTR_T *utr)
{
    UAC_DEV_T       *uac;
    unsigned int    i;
    int             ret;

    if (!utr || !utr->udev)
    {
        return;
    }

    uac = (UAC_DEV_T *)utr->context;

    /* We don't want to do anything if we are about to be removed! */
    if (!uac || !uac->udev)
    {
        return;
    }

    if (uac->asif_in.flag_streaming == 0U)
    {
        return;
    }

    //UAC_DBGMSG("SF=%d, 0x%x\n", utr->iso_sf, (int)utr);

    utr->bIsoNewSched = 0;

    for (i = 0; i < IF_PER_UTR; i++)
    {
        if (uac->asif_in.flag_streaming == 0U)
        {
            return;
        }

        if (utr->iso_status[i] == 0)
        {
            if ((uac->func_au_in != USBNULL) && (utr->iso_xlen[i] > 0U))
            {
                uac->func_au_in(uac, utr->iso_buff[i], utr->iso_xlen[i]);
            }
        }
        else
        {
            UAC_DBGMSG("Iso %d err - %d\n", i, utr->iso_status[i]);

            if ((utr->iso_status[i] == USBH_ERR_NOT_ACCESS0) || (utr->iso_status[i] == USBH_ERR_NOT_ACCESS1))
            {
                utr->bIsoNewSched = 1;
            }
        }

        utr->iso_xlen[i] = utr->ep->wMaxPacketSize;
    }

    /**
     * @brief Re-check the udev to prevent it from being set to USBNULL during execution
     * @note This is a safety check to ensure the USB device pointer remains valid
     *       throughout the operation lifecycle
     */
    if ((!utr->udev) || (uac->asif_in.flag_streaming == 0U))
    {
        return;
    }


    /* schedule the following isochronous transfers */
    ret = usbh_iso_xfer(utr);

    if (ret < 0)
    {
        UAC_DBGMSG("usbh_iso_xfer failed!\n");
    }
}

/// @endcond HIDDEN_SYMBOLS

/**
 *  @brief  Start to receive audio data from UAC device. (Microphone)
 *  @param[in] uac        Audio Class device
 *  @param[in] func       Audio in callback function.
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uac_start_audio_in(UAC_DEV_T *uac, UAC_CB_FUNC *func)
{
    UDEV_T       *udev;
    AS_IF_T      *asif;
    IFACE_T      *iface;
    ALT_IFACE_T  *aif;
    EP_INFO_T    *ep;
    UTR_T        *utr;
    uint8_t      *buff = USBNULL;
    uint8_t      bAlternateSetting;
    unsigned int i;
    unsigned int j;
    int ret;

    if (!uac)
    {
        return UAC_RET_DEV_NOT_FOUND;
    }

    udev  = uac->udev;
    asif  = &uac->asif_in;
    iface = uac->asif_in.iface;

    if (!iface)
    {
        return UAC_RET_DEV_NOT_FOUND;
    }

    if (asif->flag_streaming)
    {
        return UAC_RET_IS_STREAMING;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Select the best alternative interface                                              */
    /*  UAC 2.0: prefer highest bit-resolution PCM alt setting                            */
    /*  UAC 1.0: fallback to maximum packet size                                          */
    /*------------------------------------------------------------------------------------*/
    if ((uac->acif.bcdADC >= UAC_VERSION_2) &&
            (usbh_uac2_find_pcm_alt(uac, iface, EP_ADDR_DIR_IN, EP_ATTR_TT_ISO, 0, &bAlternateSetting) == 0))
    {
        /* Found best PCM alt for UAC 2.0 */
    }
    else if (usbh_uac_find_max_alt(iface, EP_ADDR_DIR_IN, EP_ATTR_TT_ISO, &bAlternateSetting) != 0)
    {
        return UAC_RET_FUNC_NOT_FOUND;
    }
    else
    {
        // Found best alt for UAC 1.0
    }

    uac->func_au_in = func;

    ret = usbh_set_interface(iface, bAlternateSetting);

    if (ret < 0)
    {
        UAC_ERRMSG("Failed to set interface %d, %d! (%d)\n", iface->if_num, bAlternateSetting, ret);
        return ret;
    }

    ret = uac_parse_streaming_interface(uac, iface, bAlternateSetting);

    if (ret < 0)
    {
        return ret;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find the endpoint                                                                 */
    /*------------------------------------------------------------------------------------*/
    asif->ep = USBNULL;
    aif = asif->iface->aif;

    for (i = 0; i < aif->ifd->bNumEndpoints; i++)
    {
        ep = &(aif->ep[i]);

        if (((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN) &&
                ((ep->bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_ISO))
        {
            asif->ep = ep;
            UAC_DBGMSG("Audio in endpoint 0x%x found, size: %d\n", ep->bEndpointAddress, ep->wMaxPacketSize);
            break;
        }
    }

    if (asif->ep == USBNULL)
    {
        return UAC_RET_FUNC_NOT_FOUND;
    }
    else
    {
        ep = asif->ep;
    }

#ifdef UAC_DEBUG
    UAC_DBGMSG("Activated isochronous-in endpoint =>");
    usbh_dump_ep_info(ep);
#endif

    /*------------------------------------------------------------------------------------*/
    /*  Allocate isochronous in buffer                                                    */
    /*------------------------------------------------------------------------------------*/
    for (i = 0; i < NUM_UTR; i++)           /* allocate UTRs                              */
    {
        asif->utr[i] = alloc_utr(udev);     /* allocate UTR                               */

        if (asif->utr[i] == USBNULL)
        {
            ret = USBH_ERR_MEMORY_OUT;      /* memory allocate failed                     */
            break;
        }
    }

    if (ret == USBH_ERR_MEMORY_OUT)
    {
        /* fall through to cleanup */
    }
    else
    {
        buff = (uint8_t *)usbh_alloc_mem(ep->wMaxPacketSize * IF_PER_UTR * NUM_UTR);

        if (buff == USBNULL)
        {
            ret = USBH_ERR_MEMORY_OUT;          /* memory allocate failed                     */
        }
    }

    if (ret == USBH_ERR_MEMORY_OUT)
    {
        /* fall through to cleanup */
    }
    else
    {
        for (i = 0; i < NUM_UTR; i++)           /* dispatch buffers                           */
        {
            /* divide buffer equally                      */
            utr = asif->utr[i];
            uint32_t utr_buff_offset;
            utr_buff_offset = (uint32_t)ep->wMaxPacketSize * (uint32_t)IF_PER_UTR * i;
            utr->buff = &buff[utr_buff_offset];
            utr->data_len = ep->wMaxPacketSize * IF_PER_UTR;

            for (j = 0; j < IF_PER_UTR; j++)
            {
                uint32_t iso_buff_offset;
                iso_buff_offset = (uint32_t)ep->wMaxPacketSize * j;
                utr->iso_xlen[j] = ep->wMaxPacketSize;
                utr->iso_buff[j] = &utr->buff[iso_buff_offset];
            }
        }

        /*------------------------------------------------------------------------------------*/
        /*  Start UTRs                                                                        */
        /*------------------------------------------------------------------------------------*/

        asif->utr[0]->bIsoNewSched = 1;

        for (i = 0; i < NUM_UTR; i++)
        {
            utr = asif->utr[i];
            utr->context = uac;
            utr->ep = ep;
            utr->func = uac_iso_in_irq;
            ret = usbh_iso_xfer(utr);

            if (ret < 0)
            {
                UAC_DBGMSG("Error - failed to start UTR %d isochronous-in transfer (%d)", i, ret);
                break;
            }
        }
    }

    if (ret < 0)
    {
        for (i = 0; i < NUM_UTR; i++)           /* quit all UTRs                              */
        {
            if (asif->utr[i])
            {
                (void)usbh_quit_utr(asif->utr[i]);
            }
        }

        asif->flag_streaming = 0;

        /* free USB transfer buffer                   */
        if ((asif->utr[0] != USBNULL) &&
                (asif->utr[0]->buff != USBNULL))
        {
            (void)usbh_free_mem(asif->utr[0]->buff, asif->utr[0]->data_len * NUM_UTR);
        }

        for (i = 0; i < NUM_UTR; i++)           /* free all UTRs                              */
        {
            if (asif->utr[i])
            {
                free_utr(asif->utr[i]);
            }

            asif->utr[i] = USBNULL;
        }

        return ret;
    }

    asif->flag_streaming = 1;
    uac->state = UAC_STATE_RUNNING;

    return UAC_RET_OK;
}

/**
 *  @brief  Stop UAC device audio in data stream.
 *  @param[in] uac      Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uac_stop_audio_in(UAC_DEV_T *uac)
{
    AS_IF_T      *asif = &uac->asif_in;
    const UDEV_T       *udev;
    unsigned int i;

    udev = uac->udev;

    if (!udev)
    {
        return UAC_RET_DEV_NOT_FOUND;
    }


    asif->flag_streaming = 0;

    for (i = 0; i < NUM_UTR; i++)
    {
        if (asif->utr[i])
        {
            asif->utr[i]->udev = USBNULL;  // Prevent td_done from calling callback
        }
    }

    if (asif->ep && asif->ep->hw_pipe)
    {
        (void)usbh_quit_xfer(uac->udev, asif->ep);  // Pass EP pointer to ensure ED is added to remove list
    }

    /* Set interface alternative settings */
    if (uac->state != UAC_STATE_DISCONNECTING)
    {
        int ret;
        ret = usbh_set_interface(asif->iface, 0);

        if (ret < 0)
        {
            UAC_ERRMSG("Failed to set interface %d, %d! (%d)\n", asif->iface->if_num, 0, ret);
        }
    }


    if ((asif->utr[0] != USBNULL) &&
            (asif->utr[0]->buff != USBNULL))   /* free audio buffer                          */
    {
        (void)usbh_free_mem(asif->utr[0]->buff, asif->utr[0]->data_len * NUM_UTR);
    }

    for (i = 0; i < NUM_UTR; i++)           /* free all UTRs                              */
    {
        if (asif->utr[i])
        {
            free_utr(asif->utr[i]);
        }

        asif->utr[i] = USBNULL;
    }

    if (uac->state != UAC_STATE_DISCONNECTING)
    {
        if ((uac->asif_out.iface == USBNULL) || (uac->asif_out.flag_streaming == 0U))
        {
            uac->state = UAC_STATE_READY;
        }
    }

    return UAC_RET_OK;
}

/// @cond HIDDEN_SYMBOLS

static void uac_iso_out_irq(UTR_T *utr)
{
    UAC_DEV_T   *uac;
    unsigned int i;
    int ret;

    if (!utr || !utr->udev)
    {
        return;
    }

    uac = (UAC_DEV_T *)utr->context;

    /* We don't want to do anything if we are about to be removed! */
    if (!uac || !uac->udev)
    {
        return;
    }

    if (uac->asif_out.flag_streaming == 0U)
    {
        return;
    }

    //UAC_DBGMSG("SF=%d, 0x%x\n", utr->iso_sf, (int)utr);

    utr->bIsoNewSched = 0;

    for (i = 0; i < IF_PER_UTR; i++)
    {
        if (uac->asif_out.flag_streaming == 0U)
        {
            return;
        }

        if (utr->iso_status[i] != 0)
        {
            // UAC_DBGMSG("Iso %d err - %d\n", i, utr->iso_status[i]);
            if ((utr->iso_status[i] == USBH_ERR_NOT_ACCESS0) || (utr->iso_status[i] == USBH_ERR_NOT_ACCESS1))
            {
                utr->bIsoNewSched = 1;
            }
        }

        utr->iso_xlen[i] = uac->func_au_out(uac, utr->iso_buff[i], utr->ep->wMaxPacketSize);
    }

    if ((!utr->udev) || (uac->asif_out.flag_streaming == 0U))
    {
        return;
    }

    /* schedule the following isochronous transfers */
    ret = usbh_iso_xfer(utr);

    if (ret < 0)
    {
        UAC_DBGMSG("usbh_iso_xfer failed!\n");
    }
}

/// @endcond HIDDEN_SYMBOLS

/**
 *  @brief  Start to transmit audio data to UAC device. (Speaker)
 *  @param[in] uac      Audio Class device
 *  @param[in] func     Audio out call-back function. UAC driver call this function to get audio
 *                      out stream data from user application.
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uac_start_audio_out(UAC_DEV_T *uac, UAC_CB_FUNC *func)
{
    UDEV_T       *udev;
    AS_IF_T      *asif;
    IFACE_T      *iface;
    ALT_IFACE_T  *aif;
    EP_INFO_T    *ep;
    UTR_T        *utr;
    uint8_t      *buff = USBNULL;
    uint8_t      bAlternateSetting;
    unsigned int i;
    unsigned int j;
    int          ret;

    if (!uac || !func)
    {
        return UAC_RET_DEV_NOT_FOUND;
    }

    udev  = uac->udev;
    asif  = &uac->asif_out;
    iface = uac->asif_out.iface;

    if (!iface)
    {
        return UAC_RET_DEV_NOT_FOUND;
    }

    if (asif->flag_streaming)
    {
        return UAC_RET_IS_STREAMING;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Select the best alternative interface                                              */
    /*  UAC 2.0: prefer highest bit-resolution PCM alt, constrained by peer IN bit depth   */
    /*  UAC 1.0: fallback to maximum packet size                                          */
    /*------------------------------------------------------------------------------------*/
    {

        if ((uac->acif.bcdADC >= UAC_VERSION_2) &&
                (usbh_uac2_find_pcm_alt(uac, iface, EP_ADDR_DIR_OUT, EP_ATTR_TT_ISO, 0, &bAlternateSetting) == 0))
        {
            /* Found best PCM alt for UAC 2.0 (constrained to peer IN bit depth) */
        }
        else if (usbh_uac_find_max_alt(iface, EP_ADDR_DIR_OUT, EP_ATTR_TT_ISO, &bAlternateSetting) != 0)
        {
            return UAC_RET_FUNC_NOT_FOUND;
        }
        else
        {
            // fallback to max alt for UAC 1.0
        }
    }

    uac->func_au_out = func;

    ret = usbh_set_interface(iface, bAlternateSetting);

    if (ret < 0)
    {
        UAC_ERRMSG("Failed to set interface %d, %d! (%d)\n", iface->if_num, bAlternateSetting, ret);
        return ret;
    }

    ret = uac_parse_streaming_interface(uac, iface, bAlternateSetting);

    if (ret < 0)
    {
        return ret;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find the endpoint                                                                 */
    /*------------------------------------------------------------------------------------*/
    asif->ep = USBNULL;
    aif = asif->iface->aif;

    for (i = 0; i < aif->ifd->bNumEndpoints; i++)
    {
        ep = &(aif->ep[i]);

        if (((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT) &&
                ((ep->bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_ISO))
        {
            asif->ep = ep;
            UAC_DBGMSG("Audio in endpoint 0x%x found, size: %d\n", ep->bEndpointAddress, ep->wMaxPacketSize);
            break;
        }
    }

    if (asif->ep == USBNULL)
    {
        return UAC_RET_FUNC_NOT_FOUND;
    }
    else
    {
        ep = asif->ep;
    }

#ifdef UAC_DEBUG
    UAC_DBGMSG("Activated isochronous-out endpoint =>");
    usbh_dump_ep_info(ep);
#endif

    /*------------------------------------------------------------------------------------*/
    /*  Allocate isochronous in buffer                                                    */
    /*------------------------------------------------------------------------------------*/
    for (i = 0; i < NUM_UTR; i++)           /* allocate UTRs                              */
    {
        asif->utr[i] = alloc_utr(udev);     /* allocate UTR                               */

        if (asif->utr[i] == USBNULL)
        {
            ret = USBH_ERR_MEMORY_OUT;      /* memory allocate failed                     */
            break;
        }
    }

    if (ret == USBH_ERR_MEMORY_OUT)
    {
        /* fall through to cleanup */
    }
    else
    {
        buff = (uint8_t *)usbh_alloc_mem(ep->wMaxPacketSize * IF_PER_UTR * NUM_UTR);

        if (buff == USBNULL)
        {
            ret = USBH_ERR_MEMORY_OUT;          /* memory allocate failed                     */
        }
    }

    if (ret == USBH_ERR_MEMORY_OUT)
    {
        /* fall through to cleanup */
    }
    else
    {
        for (i = 0; i < NUM_UTR; i++)           /* dispatch buffers                           */
        {
            /* divide buffer equally                      */
            uint32_t utr_buff_offset;
            utr_buff_offset = (uint32_t)ep->wMaxPacketSize * (uint32_t)IF_PER_UTR * i;
            asif->utr[i]->buff = &buff[utr_buff_offset];
            asif->utr[i]->data_len = ep->wMaxPacketSize * IF_PER_UTR;
        }

        /*------------------------------------------------------------------------------------*/
        /*  Start UTRs                                                                        */
        /*------------------------------------------------------------------------------------*/

        asif->utr[0]->bIsoNewSched = 1;

        for (i = 0; i < NUM_UTR; i++)
        {
            utr = asif->utr[i];
            utr->context = uac;
            utr->ep = ep;
            utr->func = uac_iso_out_irq;

            for (j = 0; j < IF_PER_UTR; j++)    /* get audio out data from user               */
            {
                uint32_t iso_buff_offset;
                iso_buff_offset = (uint32_t)ep->wMaxPacketSize * j;
                utr->iso_buff[j] = &utr->buff[iso_buff_offset];
                utr->iso_xlen[j] = uac->func_au_out(uac, utr->iso_buff[j], ep->wMaxPacketSize);
            }

            ret = usbh_iso_xfer(utr);

            if (ret < 0)
            {
                UAC_DBGMSG("Error - failed to start UTR %d isochronous-in transfer (%d)", i, ret);
                break;
            }
        }
    }

    if (ret < 0)
    {
        for (i = 0; i < NUM_UTR; i++)       /* quit all UTRs                              */
        {
            if (asif->utr[i])
            {
                (void)usbh_quit_utr(asif->utr[i]);
            }
        }

        asif->flag_streaming = 0;

        if ((asif->utr[0] != USBNULL) &&           /* free USB transfer buffer                   */
                (asif->utr[0]->buff != USBNULL))
        {
            (void)usbh_free_mem(asif->utr[0]->buff, asif->utr[0]->data_len * NUM_UTR);
        }

        for (i = 0; i < NUM_UTR; i++)           /* free all UTRs                              */
        {
            if (asif->utr[i])
            {
                free_utr(asif->utr[i]);
            }

            asif->utr[i] = USBNULL;
        }

        return ret;
    }

    asif->flag_streaming = 1;
    uac->state = UAC_STATE_RUNNING;

    return UAC_RET_OK;
}

/**
 *  @brief  Stop UAC device audio out data stream.
 *  @param[in] uac      Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uac_stop_audio_out(UAC_DEV_T *uac)
{
    AS_IF_T      *asif = &uac->asif_out;
    unsigned int i;

    for (i = 0; i < NUM_UTR; i++)
    {
        if (asif->utr[i])
        {
            asif->utr[i]->udev = USBNULL;  // Prevent td_done from calling callback
        }
    }

    if (asif->ep && asif->ep->hw_pipe)
    {
        (void)usbh_quit_xfer(uac->udev, asif->ep);
    }

    /* Set interface alternative settings */
    if (uac->state != UAC_STATE_DISCONNECTING)
    {
        int          ret;
        ret = usbh_set_interface(asif->iface, 0);

        if (ret < 0)
        {
            UAC_ERRMSG("Failed to set interface %d, %d! (%d)\n", asif->iface->if_num, 0, ret);
        }
    }

    if ((asif->utr[0] != USBNULL) &&
            (asif->utr[0]->buff != USBNULL))   /* free audio buffer                          */
    {
        (void)usbh_free_mem(asif->utr[0]->buff, asif->utr[0]->data_len * NUM_UTR);
    }

    for (i = 0; i < NUM_UTR; i++)           /* free all UTRs                              */
    {
        if (asif->utr[i])
        {
            free_utr(asif->utr[i]);
        }

        asif->utr[i] = USBNULL;
    }

    if (uac->state != UAC_STATE_DISCONNECTING)
    {
        if ((uac->asif_in.iface == USBNULL) || (uac->asif_in.flag_streaming == 0U))
        {
            uac->state = UAC_STATE_READY;
        }
    }

    asif->flag_streaming = 0;

    return UAC_RET_OK;
}

/**
 *  @brief   Open an connected UAC device.
 *  @param[in] uac        Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uac_open(UAC_DEV_T *uac)
{
    IFACE_T      *iface;
    uint8_t      bAlternateSetting;
    int          ret;

    /*------------------------------------------------------------------------------------*/
    /*  Select the best alternative interface for audio IN                                 */
    /*------------------------------------------------------------------------------------*/
    iface = uac->asif_in.iface;

    if (iface != USBNULL)
    {
        if ((uac->acif.bcdADC >= UAC_VERSION_2) &&
                (usbh_uac2_find_pcm_alt(uac, iface, EP_ADDR_DIR_IN, EP_ATTR_TT_ISO, 0, &bAlternateSetting) == 0))
        {
            /* Found best PCM alt for UAC 2.0 */
        }
        else if (usbh_uac_find_max_alt(iface, EP_ADDR_DIR_IN, EP_ATTR_TT_ISO, &bAlternateSetting) != 0)
        {
            return UAC_RET_FUNC_NOT_FOUND;
        }
        else
        {
            //don't do nothing.
        }

        ret = usbh_set_interface(iface, bAlternateSetting);

        if (ret < 0)
        {
            UAC_ERRMSG("Failed to set interface %d, %d! (%d)\n", iface->if_num, bAlternateSetting, ret);
            return ret;
        }

        /* Parse IN streaming descriptors so ft2/as2_gen are populated for peer matching */
        ret = uac_parse_streaming_interface(uac, iface, bAlternateSetting);

        if (ret < 0)
        {
            return ret;
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  Select the best alternative interface for audio OUT                                */
    /*  Constrain to peer IN's bit depth to avoid format conversion in application         */
    /*------------------------------------------------------------------------------------*/
    iface = uac->asif_out.iface;

    if (iface != USBNULL)
    {
        uint8_t  peer_bitres = 0;

        /* Get IN's bit resolution (parsed from the alt we just selected above) */
        if (uac->asif_in.ft2)
        {
            peer_bitres = uac->asif_in.ft2->bBitResolution;
        }

        if ((uac->acif.bcdADC >= UAC_VERSION_2) &&
                (usbh_uac2_find_pcm_alt(uac, iface, EP_ADDR_DIR_OUT, EP_ATTR_TT_ISO, peer_bitres, &bAlternateSetting) == 0))
        {
            /* Found best PCM alt for UAC 2.0 (constrained to peer IN bit depth) */
        }
        else if (usbh_uac_find_max_alt(iface, EP_ADDR_DIR_OUT, EP_ATTR_TT_ISO, &bAlternateSetting) != 0)
        {
            return UAC_RET_FUNC_NOT_FOUND;
        }
        else
        {
            //don't do nothing.
        }

        ret = usbh_set_interface(iface, bAlternateSetting);

        if (ret < 0)
        {
            UAC_ERRMSG("Failed to set interface %d, %d! (%d)\n", iface->if_num, bAlternateSetting, ret);
            return ret;
        }
    }

    return 0;
}

/** @} end of group USBH_EXPORTED_FUNCTIONS */

/** @} end of group USBH_Library */

/** @} end of group LIBRARY */
