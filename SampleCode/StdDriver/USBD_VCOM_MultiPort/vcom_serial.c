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
    uint32_t u32EpIntSts = USBD_GET_EP_INT_FLAG();
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
#if (VCOM_CNT >= 1)

                if (g_usbd_SetupPacket[4] == 0) /* VCOM-1 */
                    VCOM_LineCoding(0); /* Apply UART settings */

#endif

#if (VCOM_CNT >= 2)

                if (g_usbd_SetupPacket[4] == 2) /* VCOM-2 */
                    VCOM_LineCoding(1); /* Apply UART settings */

#endif

#if (VCOM_CNT >= 3)

                if (g_usbd_SetupPacket[4] == 4) /* VCOM-3 */
                    VCOM_LineCoding(2); /* Apply UART settings */

#endif

#if (VCOM_CNT >= 4)

                if (g_usbd_SetupPacket[4] == 6) /* VCOM-4 */
                    VCOM_LineCoding(3); /* Apply UART settings */

#endif

#if (VCOM_CNT >= 5)

                if (g_usbd_SetupPacket[4] == 8) /* VCOM-5 */
                    VCOM_LineCoding(4); /* Apply UART settings */

#endif

#if (VCOM_CNT >= 6)

                if (g_usbd_SetupPacket[4] == 10) /* VCOM-6 */
                    VCOM_LineCoding(5); /* Apply UART settings */

#endif

#if (VCOM_CNT >= 7)

                if (g_usbd_SetupPacket[4] == 12) /* VCOM-7 */
                    VCOM_LineCoding(6); /* Apply UART settings */

#endif
            }
        }

#if (VCOM_CNT >= 1)

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
        }

#endif

#if (VCOM_CNT >= 2)

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
            // Bulk Out
            EP6_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
            // Bulk IN
            EP7_Handler();

        }

#endif

#if (VCOM_CNT >= 3)

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
            // Bulk Out
            EP9_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
            // Bulk IN
            EP10_Handler();
        }

#endif

#if (VCOM_CNT >= 4)

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP12)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP12);
            // Bulk Out
            EP12_Handler();
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP13)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP13);
            // Bulk IN
            EP13_Handler();
        }

#endif

#if (VCOM_CNT >= 5)

        if (u32EpIntSts & USBD_EPINTSTS_EP14)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP14);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP15)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP15);
            // Bulk Out
            EP15_Handler();
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP16)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP16);
            // Bulk IN
            EP16_Handler();
        }

#endif

#if (VCOM_CNT >= 6)

        if (u32EpIntSts & USBD_EPINTSTS_EP17)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP17);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP18)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP18);
            // Bulk Out
            EP18_Handler();
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP19)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP19);
            // Bulk IN
            EP19_Handler();
        }

#endif

#if (VCOM_CNT >= 7)

        if (u32EpIntSts & USBD_EPINTSTS_EP20)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP20);
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP21)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP21);
            // Bulk Out
            EP21_Handler();
        }

        if (u32EpIntSts & USBD_EPINTSTS_EP22)
        {
            /* Clear event flag */
            USBD_CLR_EP_INT_FLAG(USBD_EPINTSTS_EP22);
            // Bulk IN
            EP22_Handler();
        }

#endif
    }
}

#if (VCOM_CNT >= 1)
void EP2_Handler(void)
{
    g_u32TxSize0 = 0;
}

void EP3_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize0 = USBD_GET_PAYLOAD_LEN(EP3);
    g_pu8RxBuf0 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady0 = 1;
}
#endif

#if (VCOM_CNT >= 2)
void EP6_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize1 = USBD_GET_PAYLOAD_LEN(EP6);
    g_pu8RxBuf1 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP6));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady1 = 1;
}

void EP7_Handler(void)
{
    g_u32TxSize1 = 0;
}
#endif

#if (VCOM_CNT >= 3)
void EP9_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize2 = USBD_GET_PAYLOAD_LEN(EP9);
    g_pu8RxBuf2 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP9));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady2 = 1;
}

void EP10_Handler(void)
{
    g_u32TxSize2 = 0;
}
#endif

#if (VCOM_CNT >= 4)
void EP12_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize3 = USBD_GET_PAYLOAD_LEN(EP12);
    g_pu8RxBuf3 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP12));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady3 = 1;
}

void EP13_Handler(void)
{
    g_u32TxSize3 = 0;
}
#endif

#if (VCOM_CNT >= 5)
void EP15_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize4 = USBD_GET_PAYLOAD_LEN(EP15);
    g_pu8RxBuf4 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP15));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady4 = 1;
}

void EP16_Handler(void)
{
    g_u32TxSize4 = 0;
}
#endif

#if (VCOM_CNT >= 6)
void EP18_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize5 = USBD_GET_PAYLOAD_LEN(EP18);
    g_pu8RxBuf5 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP18));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady5 = 1;
}

void EP19_Handler(void)
{
    g_u32TxSize5 = 0;
}
#endif

#if (VCOM_CNT >= 7)
void EP21_Handler(void)
{
    /* Bulk OUT */
    g_u32RxSize6 = USBD_GET_PAYLOAD_LEN(EP21);
    g_pu8RxBuf6 = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP21));

    /* Set a flag to indicate bulk out ready */
    g_i8BulkOutReady6 = 1;
}

void EP22_Handler(void)
{
    g_u32TxSize6 = 0;
}
#endif

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
#if (VCOM_CNT >= 1)
    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);
#endif

#if (VCOM_CNT >= 2)
    /*****************************************************/
    /* EP5 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_1);
    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /* EP6 ==> Bulk Out endpoint, address 4 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_1);
    /* Buffer offset for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);

    /* EP7 ==> Bulk IN endpoint, address 4 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_1);
    /* Buffer offset for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);
#endif

#if (VCOM_CNT >= 3)
    /*****************************************************/
    /* EP8 ==> Interrupt IN endpoint, address 5 */
    USBD_CONFIG_EP(EP8, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_2);
    /* Buffer offset for EP8 */
    USBD_SET_EP_BUF_ADDR(EP8, EP8_BUF_BASE);

    /* EP9 ==> Bulk Out endpoint, address 6 */
    USBD_CONFIG_EP(EP9, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_2);
    /* Buffer offset for EP9 */
    USBD_SET_EP_BUF_ADDR(EP9, EP9_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP9, EP9_MAX_PKT_SIZE);

    /* EP10 ==> Bulk IN endpoint, address 6 */
    USBD_CONFIG_EP(EP10, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_2);
    /* Buffer offset for EP10 */
    USBD_SET_EP_BUF_ADDR(EP10, EP10_BUF_BASE);
#endif

#if (VCOM_CNT >= 4)
    /*****************************************************/
    /* EP11 ==> Interrupt IN endpoint, address 7 */
    USBD_CONFIG_EP(EP11, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_3);
    /* Buffer offset for EP11 */
    USBD_SET_EP_BUF_ADDR(EP11, EP11_BUF_BASE);

    /* EP12 ==> Bulk Out endpoint, address 8 */
    USBD_CONFIG_EP(EP12, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_3);
    /* Buffer offset for EP12 */
    USBD_SET_EP_BUF_ADDR(EP12, EP12_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP12, EP12_MAX_PKT_SIZE);

    /* EP13 ==> Bulk IN endpoint, address 8 */
    USBD_CONFIG_EP(EP13, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_3);
    /* Buffer offset for EP13 */
    USBD_SET_EP_BUF_ADDR(EP13, EP13_BUF_BASE);
#endif

#if (VCOM_CNT >= 5)
    /*****************************************************/
    /* EP14 ==> Interrupt IN endpoint, address 9 */
    USBD_CONFIG_EP(EP14, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_4);
    /* Buffer offset for EP14 */
    USBD_SET_EP_BUF_ADDR(EP14, EP14_BUF_BASE);

    /* EP15 ==> Bulk Out endpoint, address 10 */
    USBD_CONFIG_EP(EP15, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_4);
    /* Buffer offset for EP15 */
    USBD_SET_EP_BUF_ADDR(EP15, EP15_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP15, EP15_MAX_PKT_SIZE);

    /* EP16 ==> Bulk IN endpoint, address 10 */
    USBD_CONFIG_EP(EP16, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_4);
    /* Buffer offset for EP16 */
    USBD_SET_EP_BUF_ADDR(EP16, EP16_BUF_BASE);
#endif

#if (VCOM_CNT >= 6)
    /* EP17 ==> Interrupt IN endpoint, address 11 */
    USBD_CONFIG_EP(EP17, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_5);
    /* Buffer offset for EP17 */
    USBD_SET_EP_BUF_ADDR(EP17, EP17_BUF_BASE);

    /* EP18 ==> Bulk Out endpoint, address 12 */
    USBD_CONFIG_EP(EP18, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_5);
    /* Buffer offset for EP18 */
    USBD_SET_EP_BUF_ADDR(EP18, EP18_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP18, EP18_MAX_PKT_SIZE);

    /* EP19 ==> Bulk IN endpoint, address 12 */
    USBD_CONFIG_EP(EP19, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_5);
    /* Buffer offset for EP19 */
    USBD_SET_EP_BUF_ADDR(EP19, EP19_BUF_BASE);
#endif

#if (VCOM_CNT >= 7)
    /* EP20 ==> Interrupt IN endpoint, address 13 */
    USBD_CONFIG_EP(EP20, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM_6);
    /* Buffer offset for EP20 */
    USBD_SET_EP_BUF_ADDR(EP20, EP20_BUF_BASE);

    /* EP21 ==> Bulk Out endpoint, address 14 */
    USBD_CONFIG_EP(EP21, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_6);
    /* Buffer offset for EP21 */
    USBD_SET_EP_BUF_ADDR(EP21, EP21_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP21, EP21_MAX_PKT_SIZE);

    /* EP22 ==> Bulk IN endpoint, address 14 */
    USBD_CONFIG_EP(EP22, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_6);
    /* Buffer offset for EP22 */
    USBD_SET_EP_BUF_ADDR(EP22, EP22_BUF_BASE);
#endif
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
#if (VCOM_CNT >= 1)

                if (au8Buf[4] == 0)   /* VCOM-1 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding0, 7);
                }

#endif

#if (VCOM_CNT >= 2)

                if (au8Buf[4] == 2)   /* VCOM-2 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding1, 7);
                }

#endif

#if (VCOM_CNT >= 3)

                if (au8Buf[4] == 4)   /* VCOM-3 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding2, 7);
                }

#endif

#if (VCOM_CNT >= 4)

                if (au8Buf[4] == 6)   /* VCOM-4 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding3, 7);
                }

#endif

#if (VCOM_CNT >= 5)

                if (au8Buf[4] == 8)   /* VCOM-5 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding4, 7);
                }

#endif

#if (VCOM_CNT >= 6)

                if (au8Buf[4] == 10)   /* VCOM-6 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding5, 7);
                }

#endif

#if (VCOM_CNT >= 7)

                if (au8Buf[4] == 12)   /* VCOM-7 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_LineCoding6, 7);
                }

#endif
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
#if (VCOM_CNT >= 1)

                if (au8Buf[4] == 0)   /* VCOM-1 */
                {
                    g_u16CtrlSignal0 = au8Buf[3];
                    g_u16CtrlSignal0 = (uint16_t)(g_u16CtrlSignal0 << 8) | au8Buf[2];
                    //printf("RTS=%d  DTR=%d\n", (g_u16CtrlSignal0 >> 1) & 1, g_u16CtrlSignal0 & 1);
                }

#endif

#if (VCOM_CNT >= 2)

                if (au8Buf[4] == 2)   /* VCOM-2 */
                {
                    g_u16CtrlSignal1 = au8Buf[3];
                    g_u16CtrlSignal1 = (uint16_t)(g_u16CtrlSignal1 << 8) | au8Buf[2];
                    //printf("RTS=%d  DTR=%d\n", (g_u16CtrlSignal1 >> 1) & 1, g_u16CtrlSignal1 & 1);
                }

#endif

#if (VCOM_CNT >= 3)

                if (au8Buf[4] == 4)   /* VCOM-3 */
                {
                    g_u16CtrlSignal2 = au8Buf[3];
                    g_u16CtrlSignal2 = (uint16_t)(g_u16CtrlSignal2 << 8) | au8Buf[2];
                    //printf("RTS=%d  DTR=%d\n", (g_u16CtrlSignal1 >> 1) & 1, g_u16CtrlSignal1 & 1);
                }

#endif

#if (VCOM_CNT >= 4)

                if (au8Buf[4] == 6)   /* VCOM-4 */
                {
                    g_u16CtrlSignal3 = au8Buf[3];
                    g_u16CtrlSignal3 = (uint16_t)(g_u16CtrlSignal3 << 8) | au8Buf[2];
                }

#endif

#if (VCOM_CNT >= 5)

                if (au8Buf[4] == 8)   /* VCOM-5 */
                {
                    g_u16CtrlSignal4 = au8Buf[3];
                    g_u16CtrlSignal4 = (uint16_t)(g_u16CtrlSignal4 << 8) | au8Buf[2];
                }

#endif

#if (VCOM_CNT >= 6)

                if (au8Buf[4] == 10)   /* VCOM-6 */
                {
                    g_u16CtrlSignal5 = au8Buf[3];
                    g_u16CtrlSignal5 = (uint16_t)(g_u16CtrlSignal5 << 8) | au8Buf[2];
                }

#endif

#if (VCOM_CNT >= 7)

                if (au8Buf[4] == 12)   /* VCOM-7 */
                {
                    g_u16CtrlSignal6 = au8Buf[3];
                    g_u16CtrlSignal6 = (uint16_t)(g_u16CtrlSignal6 << 8) | au8Buf[2];
                }

#endif

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE:
            {
#if (VCOM_CNT >= 1)

                if (au8Buf[4] == 0) /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding0, 7);

#endif

#if (VCOM_CNT >= 2)

                if (au8Buf[4] == 2) /* VCOM-2 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding1, 7);

#endif

#if (VCOM_CNT >= 3)

                if (au8Buf[4] == 4) /* VCOM-3 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding2, 7);

#endif

#if (VCOM_CNT >= 4)

                if (au8Buf[4] == 6) /* VCOM-4 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding3, 7);

#endif

#if (VCOM_CNT >= 5)

                if (au8Buf[4] == 8) /* VCOM-5 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding4, 7);

#endif

#if (VCOM_CNT >= 6)

                if (au8Buf[4] == 10) /* VCOM-6 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding5, 7);

#endif
#if (VCOM_CNT >= 7)

                if (au8Buf[4] == 12) /* VCOM-7 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_LineCoding6, 7);

#endif

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
#if (VCOM_CNT >= 1)

    if (u8Port == 0)
    {
        NVIC_DisableIRQ(DEBUG_PORT_IRQn);
        // Reset software FIFO
        g_u16ComRbytes0 = 0;
        g_u16ComRhead0 = 0;
        g_u16ComRtail0 = 0;

        g_u16ComTbytes0 = 0;
        g_u16ComThead0 = 0;
        g_u16ComTtail0 = 0;

        // Reset hardware fifo
        DEBUG_PORT->FIFO |= (UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk);

        UART_SetLineConfig(DEBUG_PORT,
                           (g_LineCoding0.u32DTERate),
                           (g_LineCoding0.u8DataBits - 5),
                           (g_LineCoding0.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding0.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding0.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding0.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(DEBUG_PORT_IRQn);
    }

#endif

#if (VCOM_CNT >= 2)
    else if (u8Port == 1)
    {
        NVIC_DisableIRQ(UART1_IRQn);
        // Reset software FIFO
        g_u16ComRbytes1 = 0;
        g_u16ComRhead1 = 0;
        g_u16ComRtail1 = 0;

        g_u16ComTbytes1 = 0;
        g_u16ComThead1 = 0;
        g_u16ComTtail1 = 0;

        // Reset hardware fifo
        UART1->FIFO |= (UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk);

        UART_SetLineConfig(UART1,
                           (g_LineCoding1.u32DTERate),
                           (g_LineCoding1.u8DataBits - 5),
                           (g_LineCoding1.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding1.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding1.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding1.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART1_IRQn);
    }

#endif
#if (VCOM_CNT >= 3)
    else if (u8Port == 2)
    {
        NVIC_DisableIRQ(UART2_IRQn);
        // Reset software FIFO
        g_u16ComRbytes2 = 0;
        g_u16ComRhead2 = 0;
        g_u16ComRtail2 = 0;

        g_u16ComTbytes2 = 0;
        g_u16ComThead2 = 0;
        g_u16ComTtail2 = 0;

        // Reset hardware FIFO
        UART2->FIFO = (UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk);

        UART_SetLineConfig(UART2,
                           (g_LineCoding2.u32DTERate),
                           (g_LineCoding2.u8DataBits - 5),
                           (g_LineCoding2.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding2.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding2.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding2.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART2_IRQn);
    }

#endif

#if (VCOM_CNT >= 4)
    else if (u8Port == 3)
    {
        NVIC_DisableIRQ(UART3_IRQn);
        // Reset software FIFO
        g_u16ComRbytes3 = 0;
        g_u16ComRhead3 = 0;
        g_u16ComRtail3 = 0;

        g_u16ComTbytes3 = 0;
        g_u16ComThead3 = 0;
        g_u16ComTtail3 = 0;

        // Reset hardware FIFO
        UART3->FIFO = (UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk);

        UART_SetLineConfig(UART3,
                           (g_LineCoding3.u32DTERate),
                           (g_LineCoding3.u8DataBits - 5),
                           (g_LineCoding3.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding3.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding3.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding3.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART3_IRQn);
    }

#endif
#if (VCOM_CNT >= 5)
    else if (u8Port == 4)
    {
        NVIC_DisableIRQ(UART4_IRQn);
        // Reset software FIFO
        g_u16ComRbytes4 = 0;
        g_u16ComRhead4 = 0;
        g_u16ComRtail4 = 0;

        g_u16ComTbytes4 = 0;
        g_u16ComThead4 = 0;
        g_u16ComTtail4 = 0;

        // Reset hardware FIFO
        UART4->FIFO = (UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk);

        UART_SetLineConfig(UART4,
                           (g_LineCoding4.u32DTERate),
                           (g_LineCoding4.u8DataBits - 5),
                           (g_LineCoding4.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding4.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding4.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding4.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART4_IRQn);
    }

#endif
#if (VCOM_CNT >= 6)
    else if (u8Port == 5)
    {
        NVIC_DisableIRQ(UART5_IRQn);
        // Reset software FIFO
        g_u16ComRbytes5 = 0;
        g_u16ComRhead5 = 0;
        g_u16ComRtail5 = 0;

        g_u16ComTbytes5 = 0;
        g_u16ComThead5 = 0;
        g_u16ComTtail5 = 0;

        // Reset hardware FIFO
        UART5->FIFO = (UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk);

        UART_SetLineConfig(UART5,
                           (g_LineCoding5.u32DTERate),
                           (g_LineCoding5.u8DataBits - 5),
                           (g_LineCoding5.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding5.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding5.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding5.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART5_IRQn);
    }

#endif

#if (VCOM_CNT >= 7)
    else if (u8Port == 6)
    {
#if (USING_UART0 == 1)
        NVIC_DisableIRQ(UART0_IRQn);
        // Reset software FIFO
        g_u16ComRbytes6 = 0;
        g_u16ComRhead6 = 0;
        g_u16ComRtail6 = 0;

        g_u16ComTbytes6 = 0;
        g_u16ComThead6 = 0;
        g_u16ComTtail6 = 0;

        // Reset hardware FIFO
        UART0->FIFO = (UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk);

        UART_SetLineConfig(UART0,
                           (g_LineCoding6.u32DTERate),
                           (g_LineCoding6.u8DataBits - 5),
                           (g_LineCoding6.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding6.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding6.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding6.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART0_IRQn);
#else
        NVIC_DisableIRQ(UART6_IRQn);
        // Reset software FIFO
        g_u16ComRbytes6 = 0;
        g_u16ComRhead6 = 0;
        g_u16ComRtail6 = 0;

        g_u16ComTbytes6 = 0;
        g_u16ComThead6 = 0;
        g_u16ComTtail6 = 0;

        // Reset hardware FIFO
        UART6->FIFO = (UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk);

        UART_SetLineConfig(UART6,
                           (g_LineCoding6.u32DTERate),
                           (g_LineCoding6.u8DataBits - 5),
                           (g_LineCoding6.u8ParityType == 0) ? UART_PARITY_NONE :
                           (g_LineCoding6.u8ParityType == 1) ? UART_PARITY_ODD :
                           (g_LineCoding6.u8ParityType == 2) ? UART_PARITY_EVEN :
                           UART_PARITY_NONE,
                           (g_LineCoding6.u8CharFormat == 0) ? UART_STOP_BIT_1 : UART_STOP_BIT_2);

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART6_IRQn);
#endif

    }

#endif
}
