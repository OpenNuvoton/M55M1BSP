/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement a Remote Network Driver
 *           Interface Specification (RNDIS) device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "rndis.h"

#include "m55m1_emac.h"


// Our MAC address
uint8_t g_au8MacAddr[6] = {0x00, 0x00, 0x00, 0x59, 0x16, 0x88};
// Buffer for holding received packet

uint32_t u32RxAct = 0;
uint32_t u32TxCnt = 0, u32RxCnt = 0;

// Descriptor pointers holds current Tx and Rx used by IRQ handler here.
uint32_t u32CurrentTxDesc, u32CurrentRxDesc;

// allocate 5 buffers for tx and other 5 for rx.
// 1 for usb, the other 4 for emac. 4 is the descriptor number allocated in this sample
// these buffers are shared between usb and emac so no memory copy is required while
// passing buffer content between two interfaces.
#if (NVT_DCACHE_ON == 1)
    #ifdef __ICCARM__
        #pragma data_alignment=4
        NVT_DTCM uint8_t rndis_outdata[RNDIS_OUTDATA_RING_SIZE];
        NVT_DTCM uint8_t rndis_indata[EMAC_RX_DESC_SIZE + 1][1580];
    #else
        NVT_DTCM uint8_t rndis_outdata[RNDIS_OUTDATA_RING_SIZE] __attribute__((aligned(32)));
        NVT_DTCM uint8_t rndis_indata[EMAC_RX_DESC_SIZE + 1][1580] __attribute__((aligned(32)));
    #endif
#else
    #ifdef __ICCARM__
        #pragma data_alignment=4
        uint8_t rndis_outdata[EMAC_TX_DESC_SIZE + 1][1580];
        uint8_t rndis_indata[EMAC_RX_DESC_SIZE + 1][1580];
    #else
        volatile uint8_t  rndis_outdata[RNDIS_OUTDATA_RING_SIZE] __attribute__((aligned(32)));
        volatile uint8_t rndis_indata[EMAC_RX_DESC_SIZE + 1][1580] __attribute__((aligned(32)));
    #endif
#endif
//for usb
volatile uint32_t u32CurrentTxBuf = 0;
volatile uint32_t u32CurrentRxBuf = 0;


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting */
    SetDebugUartCLK();

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);

    /* Select HSOTG PHY Reference clock frequency which is from HXT */
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

    /* Set HSUSB role to HSUSBD */
    SET_HSUSBDROLE();

    /* Enable HSUSB PHY */
    SYS_Enable_HSUSB_PHY();

    /* Enable HSUSBD peripheral clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    unsigned int i;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    printf("NuMicro HSUSBD RNDIS\n");

    /* Initial M55M1 EMAC module */
    EMAC_Open(&g_au8MacAddr[0]);

    /* Lock protected registers */
    SYS_LockReg();

    HSUSBD_Open(&gsHSInfo, RNDIS_ClassRequest, NULL);
    HSUSBD_SetVendorRequest(RNDIS_VendorRequest);
    /* Endpoint configuration */
    RNDIS_Init();
    NVIC_EnableIRQ(HSUSBD_IRQn);

    for (i = 0; i < EMAC_RX_DESC_SIZE + 1; i++)
    {
        *(uint32_t *)&rndis_indata[i][0] = 0x00000001; /* message type */
        *(uint32_t *)&rndis_indata[i][8] = 0x24;       /* data offset */
    }

    /* Start transaction */
    while (1)
    {
        if (HSUSBD_IS_ATTACHED())
        {
            HSUSBD_Start();
            break;
        }
    }

    while (1)
    {
        // Rx
        // Check if there any Rx packet queued in RX descriptor, if yes, move to USBD
        if ((i = EMAC_ReceivePkt()) > 0)
        {
            RNDIS_InData(i);
            u32CurrentRxBuf = (u32CurrentRxBuf + 1) % EMAC_RX_DESC_SIZE;
            printf("Rx:%d\n", u32CurrentRxBuf);
        }

        // Tx
        RNDIS_ProcessOutData();
    }
}
