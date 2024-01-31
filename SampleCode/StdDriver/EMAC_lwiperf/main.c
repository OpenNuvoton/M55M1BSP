/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A LwIP iperf sample
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "emac.h"
#include "mii.h"

#include "lwip/tcpip.h"
#include "netif/ethernetif.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/init.h"

#if LWIP_DHCP
    #include "lwip/dhcp.h"
#endif


static unsigned char sMacAddr[6] = DEFAULT_MAC0_ADDRESS;
struct netif _netif;
extern struct pbuf *queue_try_get(void);


static err_t netif_output(struct netif *netif, struct pbuf *p)
{
    uint16_t len = 0;
    u8_t *buf = NULL;

    LINK_STATS_INC(link.xmit);

    __disable_irq();

    if ((p != NULL) && (p->tot_len != 0))
    {
        buf = EMAC_AllocatePktBuf();
        len = pbuf_copy_partial(p, buf, p->tot_len, 0);

        EMAC_TransmitPkt(buf, len);
    }

    __enable_irq();

    return ERR_OK;
}

static err_t nu_netif_init(struct netif *netif)
{
    netif->linkoutput = netif_output;
    netif->output     = etharp_output;
    netif->mtu        = 1500;
    netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;

    SMEMCPY(netif->hwaddr, sMacAddr, sizeof(netif->hwaddr));
    netif->hwaddr_len = sizeof(netif->hwaddr);

    /* Initial EMAC module */
    EMAC_Open(sMacAddr);

    return ERR_OK;
}

static err_t ProcessEMACRx(struct netif *netif)
{
    struct pbuf *p;

    /* Check for received frames, feed them to lwIP */
    __disable_irq();
    p = queue_try_get();
    __enable_irq();

    if (p != NULL)
    {
        if (netif->input(p, netif) != ERR_OK)
        {
            pbuf_free(p);
        }
    }

    /* Cyclic lwIP timers check */
    sys_check_timeouts();

    return ERR_OK;
}

void TIMER0_Init(void)
{
    CLK_EnableModuleClock(TMR0_MODULE);

    // Select module clock source
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);

    // Set timer frequency to 100HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Start Timer 0
    TIMER_Start(TIMER0);
}

void TIMER1_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);

    // Select module clock source
    CLK_SetModuleClock(TMR1_MODULE, CLK_TMRSEL_TMR1SEL_HIRC, 0);

    // Set timer frequency to 1HZ
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);

    // Start Timer 1
    TIMER_Start(TIMER1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(EMAC0_MODULE);
    SYS_ResetModule(SYS_EMAC0RST);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    SET_EMAC0_RMII_MDC_PE8();
    SET_EMAC0_RMII_MDIO_PE9();
    SET_EMAC0_RMII_TXD0_PE10();
    SET_EMAC0_RMII_TXD1_PE11();
    SET_EMAC0_RMII_TXEN_PE12();
    SET_EMAC0_RMII_REFCLK_PC8();
    SET_EMAC0_RMII_RXD0_PC7();
    SET_EMAC0_RMII_RXD1_PC6();
    SET_EMAC0_RMII_CRSDV_PA7();
    SET_EMAC0_RMII_RXERR_PA6();

    GPIO_SetSlewCtl(PE, (BIT10 | BIT11 | BIT12), GPIO_SLEWCTL_FAST0);

    /* PE.13 Set high */
    GPIO_SetMode(PE, BIT13, GPIO_MODE_OUTPUT);
    PE13 = 1;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;
    uint8_t u8DHCPInit = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Timer0 interrupt interval is 10ms */
    TIMER0_Init();

    /* Timer1 interval is 1000ms */
    TIMER1_Init();

    printf("LwIP + iPerf sample code start (HCLK %d Hz)\n", SystemCoreClock);

#if LWIP_DHCP
    /* To enable LWIP_DHCP 1 in lwipopts.h */
    IP4_ADDR(&gw, 0, 0, 0, 0);
    IP4_ADDR(&ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&netmask, 0, 0, 0, 0);
#else
    IP4_ADDR(&gw, 192, 168, 1, 1);
    IP4_ADDR(&ipaddr, 192, 168, 1, 220);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
#endif

    lwip_init();
    netif_add(&_netif, &ipaddr, &netmask, &gw, NULL, nu_netif_init, netif_input);
    _netif.name[0] = 'e';
    _netif.name[1] = '0';

    netif_set_default(&_netif);
    netif_set_up(&_netif);
    netif_set_link_up(&_netif);

#if LWIP_DHCP
    printf("DHCP starting ...\n");

    if (dhcp_start(&_netif) == ERR_OK)
    {
        while ((dhcp_supplied_address(&_netif) == 0) && (u8DHCPInit == 0))
        {
            while (u8DHCPInit == 0)
            {
                ProcessEMACRx(&_netif);

                if ((uint32_t)_netif.ip_addr.addr != 0)
                {
                    u8DHCPInit = 1; // DHCP done
                    break;
                }
            }
        }
    }
    else
    {
        printf("DHCP fail\n");

        while (1) {}
    }

#endif

    printf("IP address:      %s\n", ip4addr_ntoa(&_netif.ip_addr));
    printf("Subnet mask:     %s\n", ip4addr_ntoa(&_netif.netmask));
    printf("Default gateway: %s\n", ip4addr_ntoa(&_netif.gw));

    lwiperf_start_tcp_server_default(NULL, NULL);

    while (1)
    {
#if 0

        /* Check mii link status per second */
        if (TIMER_GetIntFlag(TIMER1) != 0)
        {
            TIMER_ClearIntFlag(TIMER1);
            /* Only enable under the circumstance cable may be plug/unplug */
            mii_link_monitor(g_gmacdev);
        }

#endif

        ProcessEMACRx(&_netif);
    }
}
