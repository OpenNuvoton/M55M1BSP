/******************************************************************************
 * @file     vcom_serial.c
 * @version  V3.00
 * @brief    USBD virtual COM sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "vcom_serial.h"

uint8_t volatile g_u8EP2Ready = 0;
uint8_t volatile g_u8EP3Ready = 0;
uint8_t volatile g_u8EP4Ready = 0;
uint8_t volatile g_u8EP5Ready = 0;
uint8_t volatile g_u8Suspend = 0;

/*--------------------------------------------------------------------------*/
/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
NVT_ITCM void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        extern uint8_t g_usbd_SetupPacket[];

        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            // control OUT
            USBD_CtrlOut();

            // In ACK of SET_LINE_CODE
            if (g_usbd_SetupPacket[1] == SET_LINE_CODE)
            {
                if (g_usbd_SetupPacket[4] == 0) /* VCOM-1 */
                    VCOM_LineCoding(0); /* Apply UART settings */
            }
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk Out
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
            // Bulk IN
            EP4_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
            // Bulk Out
            EP5_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
        }

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
        }

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }
    }
}

void EP2_Handler(void)
{
    USBD_SET_PAYLOAD_LEN(EP2, 0);
    g_u32TxSize = 0;
    g_u8EP2Ready = 1;
    g_u8EP4Ready = 0;
}

void EP3_Handler(void)
{
    g_u8EP3Ready = 1;
    g_u8EP5Ready = 0;

    /* Bulk OUT */
    g_u32RxSize = USBD_GET_PAYLOAD_LEN(EP3);
    g_pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady = 1;
}

void EP4_Handler(void)
{
    USBD_SET_PAYLOAD_LEN(EP4, 0);
    g_u32TxSize = 0;
    g_u8EP4Ready = 1;
    g_u8EP2Ready = 0;
}

void EP5_Handler(void)
{
    g_u8EP5Ready = 1;
    g_u8EP3Ready = 0;

    /* Bulk OUT */
    g_u32RxSize = USBD_GET_PAYLOAD_LEN(EP5);
    g_pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady = 1;
}

/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void VCOM_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM | USBD_CFG_DB_EN | USBD_CFG_DBTGACTIVE);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);
    /* trigger receive IN data */
    USBD_SET_PAYLOAD_LEN(EP2, 0);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM | USBD_CFG_DB_EN | USBD_CFG_DBTGACTIVE);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM | USBD_CFG_DB_EN);
    /* Buffer offset for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);
    /* trigger receive IN data */
    USBD_SET_PAYLOAD_LEN(EP4, 0);

    /* EP5 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM | USBD_CFG_DB_EN);
    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);

    /* EP6 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP6 ->  */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);
}

void VCOM_ClassRequest(void)
{
    uint8_t au8Buf[8];

    USBD_GetSetupPacket(au8Buf);

    if (au8Buf[0] & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (au8Buf[1])
        {
            case GET_LINE_CODE:
            {
                if (au8Buf[4] == 0)   /* VCOM-1 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding, 7);
                }

                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch (au8Buf[1])
        {
            case SET_CONTROL_LINE_STATE:
            {
                if (au8Buf[4] == 0)   /* VCOM-1 */
                {
                    g_u16CtrlSignal = au8Buf[3];
                    g_u16CtrlSignal = (uint16_t)(g_u16CtrlSignal << 8) | au8Buf[2];
                    //printf("RTS=%d  DTR=%d\n", (g_u16CtrlSignal >> 1) & 1, g_u16CtrlSignal & 1);
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE:
            {
                if (au8Buf[4] == 0) /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding, 7);

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);

                break;
            }

            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

void VCOM_LineCoding(uint8_t u8Port)
{
    if (u8Port == 0)
    {
        NVIC_DisableIRQ(DEBUG_PORT_IRQn);
        // Reset software FIFO
        g_u16ComRbytes = 0;
        g_u16ComRhead = 0;
        g_u16ComRtail = 0;

        g_u16ComTbytes = 0;
        g_u16ComThead = 0;
        g_u16ComTtail = 0;

        // Reset hardware fifo
        DEBUG_PORT->FIFO |= (UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk);

        UART_SetLineConfig(DEBUG_PORT,
                           (g_LineCoding.u32DTERate),
                           (g_LineCoding.u8DataBits - 5),
                           (g_LineCoding.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(DEBUG_PORT_IRQn);
    }
}
