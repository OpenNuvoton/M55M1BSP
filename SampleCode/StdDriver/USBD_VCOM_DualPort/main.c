/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to implement a USB dual virtual COM port device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

#define CRYSTAL_LESS        0
#define TRIM_INIT           (SYS_BASE+0xF40)

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_LineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* DEBUG_PORT */
static volatile uint8_t s_au8ComRbuf0[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes0 = 0;
volatile uint16_t g_u16ComRhead0 = 0;
volatile uint16_t g_u16ComRtail0 = 0;

static volatile uint8_t s_au8ComTbuf0[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes0 = 0;
volatile uint16_t g_u16ComThead0 = 0;
volatile uint16_t g_u16ComTtail0 = 0;

static uint8_t s_au8RxBuf0[64] = {0};
uint8_t *g_pu8RxBuf0 = 0;
uint32_t g_u32RxSize0 = 0;
uint32_t g_u32TxSize0 = 0;

volatile int8_t g_i8BulkOutReady0 = 0;

/* UART1 */
static volatile uint8_t s_au8ComRbuf1[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes1 = 0;
volatile uint16_t g_u16ComRhead1 = 0;
volatile uint16_t g_u16ComRtail1 = 0;

static volatile uint8_t s_au8ComTbuf1[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes1 = 0;
volatile uint16_t g_u16ComThead1 = 0;
volatile uint16_t g_u16ComTtail1 = 0;

static uint8_t s_au8RxBuf1[64] = {0};
uint8_t *g_pu8RxBuf1 = 0;
uint32_t g_u32RxSize1 = 0;
uint32_t g_u32TxSize1 = 0;

volatile int8_t g_i8BulkOutReady1 = 0;

void SetDebugUartCLK(void)
{
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)

#if (CRYSTAL_LESS)

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(DEBUG_PORT_MODULE, CLK_UARTSEL0_UART6SEL_HIRC, CLK_UARTDIV0_UART6DIV(1));

#else

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(DEBUG_PORT_MODULE, CLK_UARTSEL0_UART6SEL_HXT, CLK_UARTDIV0_UART6DIV(1));

#endif

    /* Enable UART clock */
    CLK_EnableModuleClock(DEBUG_PORT_MODULE);

    /* Reset UART module */
    SYS_ResetModule(DEBUG_PORT_RST);

#endif /* !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING) */
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

#if (!CRYSTAL_LESS)

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);

    /* Select USB clock source as APLL1/2 and USB clock divider as 2 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));

#else

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ);

    /* Enable HIRC48M clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Waiting for HIRC48M clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));

#endif

    /* Enable all GPIO module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Select UART1 clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable OTG0_ module clock */
    CLK_EnableModuleClock(OTG0_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk ;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD0_MODULE);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pins for UART1 RXD and TXD */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

}

void DEBUG_PORT_Init(void)
{
    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(DEBUG_PORT, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(SYS_UART1RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART1, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void DEBUG_PORT_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = DEBUG_PORT->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while (!(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)DEBUG_PORT->DAT;

            /* Check if buffer full */
            if (g_u16ComRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf0[g_u16ComRtail0++] = u8InChar;

                if (g_u16ComRtail0 >= RXBUFSIZE)
                    g_u16ComRtail0 = 0;

                g_u16ComRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if (g_u16ComTbytes0 && (DEBUG_PORT->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes0;

            if (i32Size >= DEBUG_PORT_FIFO_SIZE)
            {
                i32Size = DEBUG_PORT_FIFO_SIZE;
            }

            while (i32Size)
            {
                u8InChar = s_au8ComTbuf0[g_u16ComThead0++];
                DEBUG_PORT->DAT = u8InChar;

                if (g_u16ComThead0 >= TXBUFSIZE)
                    g_u16ComThead0 = 0;

                g_u16ComTbytes0--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            DEBUG_PORT->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

NVT_ITCM void UART1_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART1->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while (!(UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART1->DAT;

            /* Check if buffer full */
            if (g_u16ComRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf1[g_u16ComRtail1++] = u8InChar;

                if (g_u16ComRtail1 >= RXBUFSIZE)
                    g_u16ComRtail1 = 0;

                g_u16ComRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if (g_u16ComTbytes1 && (UART1->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes1;

            if (i32Size >= UART1_FIFO_SIZE)
            {
                i32Size = UART1_FIFO_SIZE;
            }

            while (i32Size)
            {
                u8InChar = s_au8ComTbuf1[g_u16ComThead1++];
                UART1->DAT = u8InChar;

                if (g_u16ComThead1 >= TXBUFSIZE)
                    g_u16ComThead1 = 0;

                g_u16ComTbytes1--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART1->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check whether USB is ready for next packet or not */
    if (g_u32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes0)
        {
            u32Len = g_u16ComRbytes0;

            if (u32Len > EP2_MAX_PKT_SIZE)
                u32Len = EP2_MAX_PKT_SIZE;

            for (i = 0; i < u32Len; i++)
            {
                s_au8RxBuf0[i] = s_au8ComRbuf0[g_u16ComRhead0++];

                if (g_u16ComRhead0 >= RXBUFSIZE)
                    g_u16ComRhead0 = 0;
            }

            __set_PRIMASK(1);
            g_u16ComRbytes0 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize0 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)s_au8RxBuf0, u32Len);
            USBD_SET_PAYLOAD_LEN(EP2, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP2);

            if (u32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    if (g_u32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes1)
        {
            u32Len = g_u16ComRbytes1;

            if (u32Len > EP7_MAX_PKT_SIZE)
                u32Len = EP7_MAX_PKT_SIZE;

            for (i = 0; i < u32Len; i++)
            {
                s_au8RxBuf1[i] = s_au8ComRbuf1[g_u16ComRhead1++];

                if (g_u16ComRhead1 >= RXBUFSIZE)
                    g_u16ComRhead1 = 0;
            }

            __set_PRIMASK(1);
            g_u16ComRbytes1 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize1 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)s_au8RxBuf1, u32Len);
            USBD_SET_PAYLOAD_LEN(EP7, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP7);

            if (u32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (g_i8BulkOutReady0 && (g_u32RxSize0 <= TXBUFSIZE - g_u16ComTbytes0))
    {
        for (i = 0; i < g_u32RxSize0; i++)
        {
            s_au8ComTbuf0[g_u16ComTtail0++] = g_pu8RxBuf0[i];

            if (g_u16ComTtail0 >= TXBUFSIZE)
                g_u16ComTtail0 = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes0 += g_u32RxSize0;
        __set_PRIMASK(0);

        g_u32RxSize0 = 0;
        g_i8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    if (g_i8BulkOutReady1 && (g_u32RxSize1 <= TXBUFSIZE - g_u16ComTbytes1))
    {
        for (i = 0; i < g_u32RxSize1; i++)
        {
            s_au8ComTbuf1[g_u16ComTtail1++] = g_pu8RxBuf1[i];

            if (g_u16ComTtail1 >= TXBUFSIZE)
                g_u16ComTtail1 = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes1 += g_u32RxSize1;
        __set_PRIMASK(0);

        g_u32RxSize1 = 0;
        g_i8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if (g_u16ComTbytes0)
    {
        /* Check if Tx is working */
        if ((DEBUG_PORT->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            DEBUG_PORT->DAT = s_au8ComTbuf0[g_u16ComThead0++];

            if (g_u16ComThead0 >= TXBUFSIZE)
                g_u16ComThead0 = 0;

            g_u16ComTbytes0--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            DEBUG_PORT->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }

    if (g_u16ComTbytes1)
    {
        /* Check if Tx is working */
        if ((UART1->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->DAT = s_au8ComTbuf1[g_u16ComThead1++];

            if (g_u16ComThead1 >= TXBUFSIZE)
                g_u16ComThead1 = 0;

            g_u16ComTbytes1--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    PMC_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if (PMC->PWRCTL & PMC_PWRCTL_PDEN_Msk)
        PMC->PWRCTL ^= PMC_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    DEBUG_PORT_Init();
    UART1_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|        NuMicro USB Virtual COM Dual Port Sample Code        |\n");
    printf("+-------------------------------------------------------------+\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    USBD_Start();

    NVIC_EnableIRQ(DEBUG_PORT_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);
    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while (1)
    {
#if CRYSTAL_LESS

        /* Start USB trim if it is not enabled. */
        if ((SYS->TCTL48M & SYS_TCTL48M_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

                /* Re-enable crystal-less */
                SYS->TCTL48M = 0x01;
                SYS->TCTL48M |= SYS_TCTL48M_REFCKSEL_Msk | SYS_TCTL48M_BOUNDEN_Msk | (8 << SYS_TCTL48M_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if (SYS->TISTS48M & (SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->TISTS48M = 0;

            /* Clear error flags */
            SYS->TISTS12M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;

            /* Clear SOF */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
        }

#endif

        /* Enter power down when USB suspend */
        if (g_u8Suspend)
            PowerDown();

        VCOM_TransferData();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
