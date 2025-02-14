/**************************************************************************//**
 * @file     UVC.c
 * @version  V1.00
 * @brief    HSHSUSBD driver Sample file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "UVC.h"

/*--------------------------------------------------------------------------*/
/* Global variables */
volatile uint32_t gMaxPacketSize;
volatile uint8_t gAppConnected = 0;
volatile UVC_STATUS_T uvcStatus __attribute__((aligned(4)));
/****************************************************/

static volatile uint8_t gAltInterface = 0;
static volatile uint8_t g_u8UVC_PD = 0;
static volatile uint8_t g_u8UVC_FID = 0;
static volatile UVC_INFO_T uvcInfo __attribute__((aligned(4)));
static volatile UVC_PU_INFO_T uvcPuInfo __attribute__((aligned(4))) = {0, 70, 7, -10, 10, 1, 0, 20, 2, -30, 30, 0, 0, 40, 4, 0, 50, 5, 1, 5, 3, 0, 2, 2};
static volatile VIDEOSTREAMCMDDATA  VideoStreamCmdCtlData  __attribute__((aligned(4)));

void HSUSBD_IRQHandler(void)
{
    volatile uint32_t IrqStL, IrqSt;

    IrqStL = HSUSBD->GINTSTS & HSUSBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)    return;

    /* USB interrupt */
    if (IrqStL & HSUSBD_GINTSTS_USBIF_Msk)
    {
        IrqSt = HSUSBD->BUSINTSTS & HSUSBD->BUSINTEN;

        if (IrqSt & HSUSBD_BUSINTSTS_SOFIF_Msk)
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SOFIF_Msk);

        if (IrqSt & HSUSBD_BUSINTSTS_RSTIF_Msk)
        {
            HSUSBD_SwReset();
            gAppConnected = 0;

            HSUSBD_ResetDMA();
            HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;
            HSUSBD->EP[EPB].EPRSPCTL = HSUSBD_EPRSPCTL_FLUSH_Msk;

            if (HSUSBD->OPER & 0x04)  /* high speed */
                UVC_InitForHighSpeed();
            else                    /* full speed */
                UVC_InitForFullSpeed();

            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_SET_ADDR(0);
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RSTIF_Msk);
            HSUSBD_CLR_CEP_INT_FLAG(0x1ffc);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_RESUMEIF_Msk)
        {
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_SUSPENDIF_Msk)
        {
            HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_SUSPENDIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_HISPDIF_Msk)
        {
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_HISPDIF_Msk);
        }

        if (IrqSt & HSUSBD_BUSINTSTS_DMADONEIF_Msk)
        {
            g_hsusbd_DmaDone = 1;
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_DMADONEIF_Msk);

            if (HSUSBD->DMACTL & HSUSBD_DMACTL_DMARD_Msk)
            {
                if (g_hsusbd_ShortPacket == 1)
                {
                    HSUSBD->EP[EPA].EPRSPCTL = (HSUSBD->EP[EPA].EPRSPCTL & 0x10) | HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
                    g_hsusbd_ShortPacket = 0;
                }
            }
        }

        if (IrqSt & HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if (IrqSt & HSUSBD_BUSINTSTS_VBUSDETIF_Msk)
        {
            if (HSUSBD_IS_ATTACHED())
            {
                /* USB Plug In */
                HSUSBD_ENABLE_USB();
            }
            else
            {
                gAppConnected = 0;
                /* USB Un-plug */
                HSUSBD_DISABLE_USB();
            }

            HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    if (IrqStL & HSUSBD_GINTSTS_CEPIF_Msk)
    {
        IrqSt = HSUSBD->CEPINTSTS & HSUSBD->CEPINTEN;

        if (IrqSt & HSUSBD_CEPINTSTS_SETUPTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPTKIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_SETUPPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_SETUPPKIF_Msk);
            HSUSBD_ProcessSetupPacket();
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_OUTTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_OUTTKIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_INTKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);

            if (!(IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk))
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_TXPKIEN_Msk);
                HSUSBD_CtrlIn();
            }
            else
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_TXPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }

            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_PINGIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_PINGIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_TXPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);

            if (g_hsusbd_CtrlInSize)
            {
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_INTKIEN_Msk);
            }
            else
            {
                if (g_hsusbd_CtrlZero == 1)
                    HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_ZEROLEN);

                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }

            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_TXPKIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_RXPKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_RXPKIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_NAKIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_NAKIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_STALLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STALLIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_ERRIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_ERRIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_STSDONEIF_Msk)
        {
            HSUSBD_UpdateDeviceState();
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_BUFFULLIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFFULLIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }

        if (IrqSt & HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk)
        {
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk);
            goto exit_HSUSBD_IRQHandler;
        }
    }

    /* interrupt in */
    if (IrqStL & HSUSBD_GINTSTS_EPAIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPA].EPINTSTS & HSUSBD->EP[EPA].EPINTEN;

        HSUSBD_ENABLE_EP_INT(EPA, 0);
        HSUSBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }

    /* interrupt out */
    if (IrqStL & HSUSBD_GINTSTS_EPBIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPB].EPINTSTS & HSUSBD->EP[EPB].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPB, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPCIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPC].EPINTSTS & HSUSBD->EP[EPC].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPC, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPDIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPD].EPINTSTS & HSUSBD->EP[EPD].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPD, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPEIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPE].EPINTSTS & HSUSBD->EP[EPE].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPE, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPFIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPF].EPINTSTS & HSUSBD->EP[EPF].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPF, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPGIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPG].EPINTSTS & HSUSBD->EP[EPG].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPG, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPHIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPH].EPINTSTS & HSUSBD->EP[EPH].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPH, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPIIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPI].EPINTSTS & HSUSBD->EP[EPI].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPI, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPJIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPJ].EPINTSTS & HSUSBD->EP[EPJ].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPJ, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPKIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPK].EPINTSTS & HSUSBD->EP[EPK].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPK, IrqSt);
    }

    if (IrqStL & HSUSBD_GINTSTS_EPLIF_Msk)
    {
        IrqSt = HSUSBD->EP[EPL].EPINTSTS & HSUSBD->EP[EPL].EPINTEN;
        HSUSBD_CLR_EP_INT_FLAG(EPL, IrqSt);
    }

exit_HSUSBD_IRQHandler:

    __ISB();
    __DSB();
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  HSUSBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void UVC_InitForHighSpeed(void)
{
    gMaxPacketSize = EPA_MAX_PKT_SIZE;
    /* EPA ==> ISO IN endpoint, address 1 */
    HSUSBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    HSUSBD_SET_MAX_PAYLOAD(EPA, EPA_MAX_PKT_SIZE);
    HSUSBD_ConfigEp(EPA, ISO_IN_EP_NUM, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_IN);

    /* EPC ==> Interrupt IN endpoint, address 3 */
    HSUSBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN);
    HSUSBD_SET_MAX_PAYLOAD(EPC, EPC_MAX_PKT_SIZE);
    HSUSBD_ConfigEp(EPC, INT_IN_EP_NUM, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_IN);
}

void UVC_InitForFullSpeed(void)
{
    gMaxPacketSize = EPA_OTHER_MAX_PKT_SIZE;
    /* EPA ==> Interrupt IN endpoint, address 1 */
    HSUSBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    HSUSBD_SET_MAX_PAYLOAD(EPA, EPA_OTHER_MAX_PKT_SIZE);
    HSUSBD_ConfigEp(EPA, ISO_IN_EP_NUM, HSUSBD_EP_CFG_TYPE_ISO, HSUSBD_EP_CFG_DIR_IN);

    /* EPC ==> Interrupt IN endpoint, address 3 */
    HSUSBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN);
    HSUSBD_SET_MAX_PAYLOAD(EPC, EPC_OTHER_MAX_PKT_SIZE);
    HSUSBD_ConfigEp(EPC, INT_IN_EP_NUM, HSUSBD_EP_CFG_TYPE_INT, HSUSBD_EP_CFG_DIR_IN);
}

static void Enable_USBD_Clock_PHY()
{
    int i;

    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */
    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSOTGPHYEN_Msk;

    for (i = 0; i < 0x1000; i++)  // delay
    {
        __NOP();
    }

    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);
}

void UVC_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    Enable_USBD_Clock_PHY();
    /* lock protected registers */
    SYS_LockReg();

    HSUSBD_Open(&gsHSInfo, UVC_ClassRequest, UVC_SetInterface);

    /* Configure USB controller */
    /* Enable USB BUS, CEP and EPA global interrupt */
    HSUSBD_ENABLE_USB_INT(HSUSBD_GINTEN_USBIEN_Msk | HSUSBD_GINTEN_CEPIEN_Msk | HSUSBD_GINTEN_EPAIEN_Msk | HSUSBD_GINTEN_EPCIEN_Msk);
    /* Enable BUS interrupt */
    HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    HSUSBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    HSUSBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);

    /*****************************************************/
    UVC_InitForHighSpeed();

    uvcInfo.u8ErrCode = 0;
    uvcInfo.bChangeData = 0;
    uvcStatus.StillImage = 0;

    uvcInfo.IsoMaxPacketSize[0] = gMaxPacketSize;
    uvcInfo.IsoMaxPacketSize[1] = gMaxPacketSize;
    uvcInfo.IsoMaxPacketSize[2] = gMaxPacketSize;

    //Initialize Video Capture Filter Control Value
    uvcInfo.CapFilter.Brightness = 1;
    uvcInfo.CapFilter.POWER_LINE_FREQUENCY = 2;
    uvcInfo.CapFilter.Brightness = 1;
    uvcInfo.CapFilter.Contrast = 2;
    uvcInfo.CapFilter.Hue = 3;
    uvcInfo.CapFilter.Saturation = 4;
    uvcInfo.CapFilter.Sharpness = 5;
    uvcInfo.CapFilter.Gamma = 3;
    uvcInfo.CapFilter.Backlight = 7;

    /* Initialize Video Probe and Commit Control data */
    memset((uint8_t *)&uvcInfo.VSCmdCtlData, 0x0, sizeof(VIDEOSTREAMCMDDATA));
    uvcInfo.VSCmdCtlData.bmHint = 0x0100;
    uvcInfo.VSCmdCtlData.bFormatIndex = 1;
    uvcInfo.VSCmdCtlData.bFrameIndex = 2;
    uvcStatus.FormatIndex = 1;
    uvcStatus.FrameIndex = 2;
    uvcInfo.VSCmdCtlData.dwFrameInterval = 0x051615;

    /* Initialize Still Image Command data */
    memset((uint8_t *)&uvcInfo.VSStillCmdCtlData, 0x0, sizeof(VIDEOSTREAMSTILLCMDDATA));

    uvcInfo.VSStillCmdCtlData.bFormatIndex = 1;
    uvcInfo.VSStillCmdCtlData.bFrameIndex = 1;
    uvcStatus.snapshotFormatIndex = 1;
    uvcStatus.snapshotFrameIndex = 1;

    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(HSUSBD_IRQn);
}

void UVC_ClassRequest(void)
{
    uint32_t volatile Data;

    if (gUsbCmd.bmRequestType & 0x80)
    {
        /* Device to host */
        /* Video Control Requests */
        /* Unit and Terminal Control Requests */
        /* Interface Control Requests */
        if (gUsbCmd.wIndex == 0x0000 && (gUsbCmd.wValue == VC_REQUEST_ERROR_CODE_CONTROL))  //Only send to Video Control interface ID(00)
        {
            HSUSBD_PrepareCtrlIn((uint8_t *)&uvcInfo.u8ErrCode, gUsbCmd.wLength);
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
        }
        //Processing Unit Control Requests
        else if (gUsbCmd.wIndex == 0x0500)  //Processing Unit ID(05) and Video Control interface ID(00)
        {
            Data = UVC_PU_Info(gUsbCmd.bRequest, gUsbCmd.wValue);
            HSUSBD_PrepareCtrlIn((uint8_t *)&uvcInfo.u8ErrCode, gUsbCmd.wLength);
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
        }
        //Camera Terminal Control Requests
        else if (gUsbCmd.wIndex == 0x0100)  //Camera Terminal ID(01) and Video Control interface ID(00)
        {
            uvcInfo.u8ErrCode = EC_Invalid_Control;
            /* STALL control pipe */
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
        }
        ///Videostreaming Requests
        else if (gUsbCmd.wIndex == 0x0001)  //VideoStreaming interface(0x01)
        {
            if ((gUsbCmd.wValue == VS_PROBE_CONTROL) || (gUsbCmd.wValue == VS_COMMIT_CONTROL))
            {
                if ((gUsbCmd.bRequest == UVC_GET_INFO) || (gUsbCmd.bRequest == UVC_GET_LEN))
                {
                    if (gUsbCmd.bRequest == UVC_GET_INFO)
                        Data = 3;//Info
                    else
                        Data = 26;//Length

                    HSUSBD_PrepareCtrlIn((uint8_t *)&Data, gUsbCmd.wLength);
                    HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                }
                else
                {
                    if ((uvcInfo.VSCmdCtlData.bFormatIndex != uvcInfo.u32FormatIndx) || (uvcInfo.VSCmdCtlData.bFrameIndex != uvcInfo.u32FrameIndx))
                    {
                        uvcStatus.FormatIndex = uvcInfo.u32FormatIndx = uvcInfo.VSCmdCtlData.bFormatIndex;
                        uvcStatus.FrameIndex = uvcInfo.u32FrameIndx = uvcInfo.VSCmdCtlData.bFrameIndex;

                        uvcInfo.bChangeData = 1;
                    }

                    switch (uvcInfo.VSCmdCtlData.bFrameIndex)
                    {
                        case UVC_QQVGA://160*120(5)
                            uvcInfo.VSCmdCtlData.dwMaxVideoFrameSize = UVC_SIZE_QQVGA;
                            break;

                        case UVC_QVGA://320*240(3)
                            uvcInfo.VSCmdCtlData.dwMaxVideoFrameSize = UVC_SIZE_QVGA;
                            break;

                        case UVC_VGA://640*480(1)
                            uvcInfo.VSCmdCtlData.dwMaxVideoFrameSize = UVC_SIZE_VGA;
                            break;
                    }

                    //************Depend on the specificed frame*****************//

                    uvcInfo.VSCmdCtlData.dwMaxPayloadTransferSize = uvcInfo.IsoMaxPacketSize[gAltInterface];

                    uvcStatus.MaxVideoFrameSize = uvcInfo.VSCmdCtlData.dwMaxVideoFrameSize;
                    uvcInfo.VSCmdCtlData.dwMaxPayloadTransferSize = gMaxPacketSize;
                    uvcInfo.VSCmdCtlData.wCompQuality = 0x3D;
                    HSUSBD_PrepareCtrlIn((uint8_t *)&uvcInfo.VSCmdCtlData, gUsbCmd.wLength);
                    HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                }
            }
            else if ((gUsbCmd.wValue == VS_STILL_PROBE_CONTROL) || (gUsbCmd.wValue == VS_STILL_COMMIT_CONTROL))
            {
                if ((gUsbCmd.bRequest == UVC_GET_INFO) || (gUsbCmd.bRequest == UVC_GET_LEN))
                {
                    if (gUsbCmd.bRequest == UVC_GET_INFO)
                        Data = 3;//Info
                    else
                        Data = 11;//Length

                    HSUSBD_PrepareCtrlIn((uint8_t *)&Data, gUsbCmd.wLength);
                    HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                }
                else
                {
                    switch (uvcInfo.VSStillCmdCtlData.bFrameIndex)
                    {
                        case UVC_STILL_QQVGA://160*120(7)
                            uvcInfo.VSStillCmdCtlData.dwMaxVideoFrameSize = UVC_SIZE_QQVGA;
                            break;

                        case UVC_STILL_QVGA://320*240(5)
                            uvcInfo.VSStillCmdCtlData.dwMaxVideoFrameSize = UVC_SIZE_QVGA;
                            break;

                        case UVC_STILL_VGA://640*480(3)
                            uvcInfo.VSStillCmdCtlData.dwMaxVideoFrameSize = UVC_SIZE_VGA;
                            break;
                    }

                    uvcStatus.snapshotFormatIndex = uvcInfo.VSStillCmdCtlData.bFormatIndex;
                    uvcStatus.snapshotFrameIndex = uvcInfo.VSStillCmdCtlData.bFrameIndex;
                    uvcStatus.snapshotMaxVideoFrameSize = uvcInfo.VSStillCmdCtlData.dwMaxPayloadTranferSize = uvcInfo.IsoMaxPacketSize[gAltInterface];
                    HSUSBD_PrepareCtrlIn((uint8_t *)&uvcInfo.VSStillCmdCtlData, gUsbCmd.wLength);
                    HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                    HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
                }

            }
            else if (gUsbCmd.wValue == VS_STILL_IMAGE_TRIGGER_CONTROL)
            {
                if (gUsbCmd.bRequest == UVC_GET_INFO)
                    Data = 3;//Info
                else
                    Data = uvcStatus.StillImage;//Trigger

                HSUSBD_PrepareCtrlIn((uint8_t *)&Data, gUsbCmd.wLength);
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_INTKIF_Msk);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_INTKIEN_Msk);
            }
        }
        else
        {
            /* Setup error, stall the device */
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
        }
    }
    else
    {
        /* Host to device */
        /* Video Probe Control Request */
        if (gUsbCmd.wIndex == 0x0500)
        {
            uint16_t OutValue = 0;

            if (gUsbCmd.bRequest == UVC_SET_CUR)
            {
                HSUSBD_CtrlOut((uint8_t *)&OutValue, gUsbCmd.wLength);
                UVC_PU_Control(OutValue);
                /* Status stage */
                HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
                HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
                HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }
        }
        else if (gUsbCmd.wIndex == 0x0100)
        {
            uvcInfo.u8ErrCode = EC_Invalid_Request;
            /* Setup error, stall the device */
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
        }
        else if (gUsbCmd.wIndex == 0x0001)
        {
            if (gUsbCmd.wValue == VS_STILL_IMAGE_TRIGGER_CONTROL)
                HSUSBD_CtrlOut((uint8_t *)&uvcStatus.StillImage, gUsbCmd.wLength);
            else if ((gUsbCmd.wValue == VS_PROBE_CONTROL) || (gUsbCmd.wValue == VS_COMMIT_CONTROL))
                HSUSBD_CtrlOut((uint8_t *)&uvcInfo.VSCmdCtlData, gUsbCmd.wLength);
            else if ((gUsbCmd.wValue == VS_STILL_PROBE_CONTROL) || (gUsbCmd.wValue == VS_STILL_COMMIT_CONTROL))
                HSUSBD_CtrlOut((uint8_t *)&uvcInfo.VSStillCmdCtlData, gUsbCmd.wLength);

            /* Status stage */
            HSUSBD_CLR_CEP_INT_FLAG(HSUSBD_CEPINTSTS_STSDONEIF_Msk);
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_NAKCLR);
            HSUSBD_ENABLE_CEP_INT(HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
        }
        else
        {
            /* Setup error, stall the device */
            HSUSBD_SET_CEP_STATE(HSUSBD_CEPCTL_STALLEN_Msk);
        }
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UVC Class relative setting
 */
void UVC_SetInterface(uint32_t u32AltInterface)
{
    gAltInterface = gAppConnected = u32AltInterface;

    if (gUsbCmd.wValue == 0)
    {
        /* stop usbd dma and flush FIFO */
        HSUSBD_ResetDMA();
        g_hsusbd_DmaDone = 1;
        HSUSBD->EP[EPA].EPRSPCTL |= HSUSBD_EPRSPCTL_FLUSH_Msk;
    }
}

uint32_t UVC_PU_Info(uint32_t Req, uint32_t Unit)
{
    uvcInfo.u8ErrCode = 0;

    switch (Req)
    {
        case UVC_GET_CUR:
            switch (Unit)
            {
                case PU_BACKLIGHT_COMPENSATION_CONTROL:
                    return uvcInfo.CapFilter.Backlight;

                case PU_BRIGHTNESS_CONTROL:
                    return uvcInfo.CapFilter.Brightness;

                case PU_CONTRAST_CONTROL:
                    return uvcInfo.CapFilter.Contrast;

                case PU_HUE_CONTROL:
                    return uvcInfo.CapFilter.Hue;

                case PU_SATURATION_CONTROL:
                    return uvcInfo.CapFilter.Saturation;

                case PU_SHARPNESS_CONTROL:
                    return uvcInfo.CapFilter.Sharpness;

                case PU_GAMMA_CONTROL:
                    return uvcInfo.CapFilter.Gamma;

                case PU_POWER_LINE_FREQUENCY_CONTROL:
                    return uvcInfo.CapFilter.POWER_LINE_FREQUENCY;

                default:
                    uvcInfo.u8ErrCode = EC_Invalid_Control;
                    return 6;
            }

        case UVC_GET_MIN:
            switch (Unit)
            {
                case PU_BACKLIGHT_COMPENSATION_CONTROL:
                    return uvcPuInfo.PU_BACKLIGHT_COMPENSATION_MIN;

                case PU_BRIGHTNESS_CONTROL:
                    return uvcPuInfo.PU_BRIGHTNESS_MIN;

                case PU_CONTRAST_CONTROL:
                    return uvcPuInfo.PU_CONTRAST_MIN;

                case PU_HUE_CONTROL:
                    return uvcPuInfo.PU_HUE_MIN;

                case PU_SATURATION_CONTROL:
                    return uvcPuInfo.PU_SATURATION_MIN;

                case PU_SHARPNESS_CONTROL:
                    return uvcPuInfo.PU_SHARPNESS_MIN;

                case PU_GAMMA_CONTROL:
                    return uvcPuInfo.PU_GAMMA_MIN;

                case PU_POWER_LINE_FREQUENCY_CONTROL:
                    return uvcPuInfo.PU_POWER_LINE_FREQUENCY_MIN;

                default:
                    uvcInfo.u8ErrCode = EC_Invalid_Control;
                    return 6;
            }

        case UVC_GET_MAX:
            switch (Unit)
            {
                case PU_BACKLIGHT_COMPENSATION_CONTROL:
                    return uvcPuInfo.PU_BACKLIGHT_COMPENSATION_MAX;

                case PU_BRIGHTNESS_CONTROL:
                    return uvcPuInfo.PU_BRIGHTNESS_MAX;

                case PU_CONTRAST_CONTROL:
                    return uvcPuInfo.PU_CONTRAST_MAX;

                case PU_HUE_CONTROL:
                    return uvcPuInfo.PU_HUE_MAX;

                case PU_SATURATION_CONTROL:
                    return uvcPuInfo.PU_SATURATION_MAX;

                case PU_SHARPNESS_CONTROL:
                    return uvcPuInfo.PU_SHARPNESS_MAX;

                case PU_GAMMA_CONTROL:
                    return uvcPuInfo.PU_GAMMA_MAX;

                case PU_POWER_LINE_FREQUENCY_CONTROL:
                    return uvcPuInfo.PU_POWER_LINE_FREQUENCY_MAX;

                default:
                    uvcInfo.u8ErrCode = EC_Invalid_Control;
                    return 6;
            }

        case UVC_GET_RES:
            return 1;

        case UVC_GET_INFO:
            switch (Unit)
            {
                case PU_BACKLIGHT_COMPENSATION_CONTROL:
                case PU_BRIGHTNESS_CONTROL:
                case PU_CONTRAST_CONTROL:
                case PU_HUE_CONTROL:
                case PU_SATURATION_CONTROL:
                case PU_SHARPNESS_CONTROL:
                case PU_GAMMA_CONTROL:
                case PU_POWER_LINE_FREQUENCY_CONTROL:
                    return 3;

                default:
                    uvcInfo.u8ErrCode = EC_Invalid_Control;
                    return 0;
            }

        case UVC_GET_DEF:
            switch (Unit)
            {
                case PU_BACKLIGHT_COMPENSATION_CONTROL:
                    return uvcPuInfo.PU_BACKLIGHT_COMPENSATION_DEF;

                case PU_BRIGHTNESS_CONTROL:
                    return uvcPuInfo.PU_BRIGHTNESS_DEF;

                case PU_CONTRAST_CONTROL:
                    return uvcPuInfo.PU_CONTRAST_DEF;

                case PU_HUE_CONTROL:
                    return uvcPuInfo.PU_HUE_DEF;

                case PU_SATURATION_CONTROL:
                    return uvcPuInfo.PU_SATURATION_DEF;

                case PU_SHARPNESS_CONTROL:
                    return uvcPuInfo.PU_SHARPNESS_DEF;

                case PU_GAMMA_CONTROL:
                    return uvcPuInfo.PU_GAMMA_DEF;

                case PU_POWER_LINE_FREQUENCY_CONTROL:
                    return uvcPuInfo.PU_POWER_LINE_FREQUENCY_DEF;

                default:
                    uvcInfo.u8ErrCode = EC_Invalid_Control;
                    return 6;
            }

        case UVC_GET_LEN:
            uvcInfo.u8ErrCode = EC_Invalid_Request;
            return 6;
    }

    return 6;
}

/* Process Unit Control */
uint32_t UVC_ProcessUnitControl(uint32_t u32ItemSelect, uint32_t u32Value)
{
    switch (u32ItemSelect)
    {
        case PU_BACKLIGHT_COMPENSATION_CONTROL:
            //printf("Set Backlight -> %d\n",u32Value);
            break;

        case PU_BRIGHTNESS_CONTROL:
            //printf("Set Brightness -> %d\n",u32Value);
            break;

        case PU_CONTRAST_CONTROL:
            //printf("Set Contrast -> %d\n",u32Value);
            break;

        case PU_HUE_CONTROL:
            //printf("Set Hue -> %d\n",u32Value);
            break;

        case PU_SATURATION_CONTROL:
            //printf("Set Saturation -> %d\n",u32Value);
            break;

        case PU_SHARPNESS_CONTROL:
            //printf("Set Sharpness -> %d\n",u32Value);
            break;

        case PU_GAMMA_CONTROL:
            //printf("Set Gamma -> %d\n",u32Value);
            break;

        case PU_POWER_LINE_FREQUENCY_CONTROL:
            //printf("Set Power Line Frequency -> %d\n",u32Value);
            break;

        default:
            ;
    }

    return 0;
}

void UVC_PU_Control(int16_t Value)
{
    uvcInfo.u8ErrCode = 0;

    switch (gUsbCmd.wValue)
    {
        case PU_BACKLIGHT_COMPENSATION_CONTROL:
            if (Value >= uvcPuInfo.PU_BACKLIGHT_COMPENSATION_MIN && Value <= uvcPuInfo.PU_BACKLIGHT_COMPENSATION_MAX)
            {
                uvcInfo.CapFilter.Backlight = Value;
                UVC_ProcessUnitControl(PU_BACKLIGHT_COMPENSATION_CONTROL, Value);
                return;
            }

            break;

        case PU_BRIGHTNESS_CONTROL:
            if (Value >= uvcPuInfo.PU_BRIGHTNESS_MIN && Value <= uvcPuInfo.PU_BRIGHTNESS_MAX)
            {
                uvcInfo.CapFilter.Brightness = Value;
                UVC_ProcessUnitControl(PU_BRIGHTNESS_CONTROL, Value);
                return;
            }

            break;

        case PU_CONTRAST_CONTROL:
            if (Value >= uvcPuInfo.PU_CONTRAST_MIN && Value <= uvcPuInfo.PU_CONTRAST_MAX)
            {
                uvcInfo.CapFilter.Contrast = Value;
                UVC_ProcessUnitControl(PU_CONTRAST_CONTROL, Value);
                return;
            }

            break;

        case PU_HUE_CONTROL:
            if (Value >= uvcPuInfo.PU_HUE_MIN && Value <= uvcPuInfo.PU_HUE_MAX)
            {
                uvcInfo.CapFilter.Hue = Value;
                UVC_ProcessUnitControl(PU_HUE_CONTROL, Value);
                return;
            }

            break;

        case PU_SATURATION_CONTROL:
            if (Value >= uvcPuInfo.PU_SATURATION_MIN && Value <= uvcPuInfo.PU_SATURATION_MAX)
            {
                uvcInfo.CapFilter.Saturation = Value;
                UVC_ProcessUnitControl(PU_SATURATION_CONTROL, Value);
                return;
            }

            break;

        case PU_SHARPNESS_CONTROL:
            if (Value >= uvcPuInfo.PU_SHARPNESS_MIN && Value <= uvcPuInfo.PU_SHARPNESS_MAX)
            {
                uvcInfo.CapFilter.Sharpness = Value;
                UVC_ProcessUnitControl(PU_SHARPNESS_CONTROL, Value);
                return;
            }

            break;

        case PU_GAMMA_CONTROL:
            if (Value >= uvcPuInfo.PU_GAMMA_MIN && Value <= uvcPuInfo.PU_GAMMA_MAX)
            {
                uvcInfo.CapFilter.Gamma = Value;
                UVC_ProcessUnitControl(PU_GAMMA_CONTROL, Value);
                return;
            }

            break;

        case PU_POWER_LINE_FREQUENCY_CONTROL:
            if (Value >= uvcPuInfo.PU_POWER_LINE_FREQUENCY_MIN && Value <= uvcPuInfo.PU_POWER_LINE_FREQUENCY_MAX)
            {
                uvcInfo.CapFilter.POWER_LINE_FREQUENCY = Value;
                UVC_ProcessUnitControl(PU_POWER_LINE_FREQUENCY_CONTROL, Value);
                return;
            }

            break;
    }

    uvcInfo.u8ErrCode = EC_Out_Of_Range;
}


void UVC_ActiveDMA(uint32_t u32Addr, uint32_t u32Len)
{
    /* Enable BUS interrupt */
    HSUSBD_ENABLE_BUS_INT(HSUSBD_BUSINTEN_DMADONEIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk | HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk);

    HSUSBD_SET_DMA_ADDR(u32Addr);
    HSUSBD_SET_DMA_LEN(u32Len);
    g_hsusbd_DmaDone = 0;
    HSUSBD_ENABLE_DMA();

    while (g_hsusbd_Configured)
    {
        if (g_hsusbd_DmaDone)
            break;

        if ((HSUSBD_IS_ATTACHED() == 0) || (gAppConnected == 0))
        {
            if (HSUSBD->DMACTL & 0x20)
            {
                HSUSBD_ResetDMA();
                HSUSBD->EP[EPA].EPRSPCTL |= HSUSBD_EPRSPCTL_FLUSH_Msk;
            }

            break;
        }
    }
}


void UVC_IsoIn(uint32_t u32Addr, uint32_t u32Len)
{
    uint32_t u32Loop, toggle = 0;
    uint32_t i, header;
    uint8_t *ptr;

    if (uvcInfo.bChangeData == 1)
    {
        uvcInfo.bChangeData = 0;

        if (HSUSBD->DMACNT & 0x20)
        {
            HSUSBD_ResetDMA();
            uvcStatus.bReady = TRUE;
        }
    }

    /* iso in, dma read, epnum = 1 */
    HSUSBD_SET_DMA_READ(ISO_IN_EP_NUM);

    ptr = (uint8_t *)u32Addr;
    u32Loop = u32Len / USBD_MAX_DMA_LEN;

    for (i = 0; i < u32Loop; i++)
    {
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk | HSUSBD_EPINTEN_SHORTTXIEN_Msk);
        g_hsusbd_ShortPacket = 0;
        /* Start of Transfer Payload Data */
        header = ((g_u8UVC_PD | (g_u8UVC_FID & 0x01)) << 8) | 0x02;

        while (1)
        {
            if (HSUSBD_GET_EP_INT_FLAG(EPA) & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
            {
                HSUSBD->EP[EPA].EPDAT_BYTE = header & 0xff;
                HSUSBD->EP[EPA].EPDAT_BYTE = (header >> 8) & 0xff;

                if (toggle)
                {
                    HSUSBD->EP[EPA].EPDAT_BYTE = *ptr & 0xff;
                    HSUSBD->EP[EPA].EPDAT_BYTE = *(ptr + 1) & 0xff;

                    UVC_ActiveDMA((uint32_t)(ptr + 2), USBD_MAX_DMA_LEN - 2);
                }
                else
                {
                    UVC_ActiveDMA((uint32_t)ptr, USBD_MAX_DMA_LEN);
                }

                toggle ^= 1;
                ptr += USBD_MAX_DMA_LEN;
                break;
            }
        }
    }

    u32Loop = u32Len % USBD_MAX_DMA_LEN;

    if (u32Loop)
    {
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_TXPKIEN_Msk | HSUSBD_EPINTEN_SHORTTXIEN_Msk);
        g_hsusbd_ShortPacket = 1;
        header = ((g_u8UVC_PD | UVC_PH_EndOfFrame | (g_u8UVC_FID & 0x01)) << 8) | 0x02; //End Of Frame

        while (1)
        {
            if (HSUSBD_GET_EP_INT_FLAG(EPA) & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
            {
                HSUSBD->EP[EPA].EPDAT_BYTE = header & 0xff;
                HSUSBD->EP[EPA].EPDAT_BYTE = (header >> 8) & 0xff;

                if (toggle)
                {
                    HSUSBD->EP[EPA].EPDAT_BYTE = *ptr & 0xff;
                    HSUSBD->EP[EPA].EPDAT_BYTE = *(ptr + 1) & 0xff;

                    UVC_ActiveDMA((uint32_t)(ptr + 2), u32Loop - 2);
                }
                else
                {
                    UVC_ActiveDMA((uint32_t)ptr, u32Loop);
                }

                g_u8UVC_FID++;
                break;
            }
        }
    }
}

int UVC_SendImage(uint32_t u32Addr, uint32_t u32transferSize, uint32_t bStillImage)
{
    if (u32transferSize != 0)
    {
        //printf("send: 0x%x, %d (%d)\n", u32Addr, u32transferSize, bStillImage);
        uvcStatus.bReady = 0;

        HSUSBD_ResetDMA();

        if (bStillImage)
        {
            g_u8UVC_PD = UVC_PH_Snapshot;       /* Payload Header for Snapshot */
            uvcStatus.StillImage = 0;
        }
        else
            g_u8UVC_PD = UVC_PH_Preview;        /* Payload Header for Preview */

        UVC_IsoIn(u32Addr, u32transferSize);

        return 1;
    }

    return 0;
}

bool UVC_IsConnect(void)
{
    if ((gAppConnected) && (HSUSBD_IS_ATTACHED()))
        return TRUE;

    return FALSE;
}

