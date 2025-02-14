/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An LwIP iperf example with dual-stack support for both IPv4 and IPv6.
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

#if LWIP_IPV6_DHCP6
    #include "lwip/dhcp6.h"
#endif

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

#if LWIP_IPV6
    extern err_t ethip6_output(struct netif *netif, struct pbuf *q, const ip6_addr_t *ip6addr);
#endif
static err_t nu_netif_init(struct netif *netif)
{
    netif->linkoutput = netif_output;
    netif->output     = etharp_output;
    netif->mtu        = 1500;
    netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
    netif->hwaddr_len = 6;

#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
    netif->ip6_autoconfig_enabled = 1;
    netif_create_ip6_linklocal_address(netif, 1);
#endif

    netif->hwaddr_len = sizeof(netif->hwaddr);

#if LWIP_IPV6 && LWIP_IPV6_MLD
    netif->flags      |= NETIF_FLAG_MLD6;

    /*
     * For hardware/netifs that implement MAC filtering.
     * All-nodes link-local is handled by default, so we must let the hardware know
     * to allow multicast packets in.
     * Should set mld_mac_filter previously. */
    if (netif->mld_mac_filter != NULL)
    {
        ip6_addr_t ip6_allnodes_ll;
        ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
        netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_ADD_MAC_FILTER);
    }

#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

    /* Initial EMAC module */
    EMAC_Open(netif->hwaddr);

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

    /* Select module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);

    /* Set timer frequency to 1000HZ */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);

    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Start Timer 0 */
    TIMER_Start(TIMER0);
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

    /* Enable PLL0 220MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

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

    /* PHY reset pin PE.13 set to high */
    GPIO_SetMode(PE, BIT13, GPIO_MODE_OUTPUT);
    PE13 = 1;

    /* Lock protected registers */
    SYS_LockReg();
}

static void netif_status_callback(struct netif *netif)
{
    if (netif)
    {
        printf("\n========================================================\n");

        printf("netif(%c%c) status changed\n", netif->name[0], netif->name[1]);
        printf("IPv4 address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
        printf("Subnet mask: %s\n", ipaddr_ntoa((const ip_addr_t *) netif_ip4_netmask(netif)));
        printf("Default gateway: %s\n", ipaddr_ntoa((const ip_addr_t *) netif_ip4_gw(netif)));

#if LWIP_IPV6

        if (netif_ip6_addr_state(netif, 0))
            printf("IPv6 address(link-local): %s\n", ipaddr_ntoa(netif_ip_addr6(netif, 0)));

        if (netif_ip6_addr_state(netif, 1))
            printf("IPv6 address(stateless address auto-configuration, SLACC): %s\n", ipaddr_ntoa(netif_ip_addr6(netif, 1)));

#endif

        printf("========================================================\n");
    }
}

int main(void)
{
    struct netif _netif;
    u8_t s_au8MACAddr[6] = DEFAULT_MAC0_ADDRESS;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Timer0 interrupt interval is 10ms */
    TIMER0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("An LwIP iperf example with dual-stack support for both IPv4 and IPv6 start (HCLK %d Hz)\n", SystemCoreClock);

    /* Initial _netif instance. */
    memset((void *)&_netif, 0, sizeof(struct netif));

    /* Copy MAC address to netif instance. */
    SMEMCPY(_netif.hwaddr, s_au8MACAddr, sizeof(_netif.hwaddr));

    /* Set interface name. */
    _netif.name[0] = 'e';
    _netif.name[1] = '0';

    lwip_init();

#if LWIP_DHCP
    /* To enable LWIP_DHCP 1 in lwipopts.h */
    netif_add_noaddr(&_netif, NULL, nu_netif_init, netif_input);
#else
    {
        ip_addr_t ipaddr, netmask, gw;
        IP4_ADDR(&gw, 192, 168, 1, 1);
        IP4_ADDR(&ipaddr, 192, 168, 1, 220);
        IP4_ADDR(&netmask, 255, 255, 255, 0);
        netif_add(&_netif, &ipaddr, &netmask, &gw, NULL, nu_netif_init, netif_input);
    }
#endif

    /* Register callback of interface status changing. */
    netif_set_status_callback(&_netif, netif_status_callback);
    netif_set_default(&_netif);
    netif_set_up(&_netif);

#if LWIP_DHCP
    printf("\n\nDHCP starting ...\n\n");

    /* Enable DHCP procedure. */
    if (dhcp_start(&_netif) != ERR_OK)
    {
        printf("\n*** dhcp_start fail. halted\n");

        while (1) {}
    }

#endif

#if LWIP_IPV6_DHCP6
    _netif.ip6_autoconfig_enabled = 1;

    /* Enable DHCPv6 procedure. */
    if (dhcp6_enable_stateless(&_netif) != ERR_OK)
    {
        printf("\n*** dhcp6_enable_stateless fail. halted\n\n");

        while (1) {}
    }

#endif

    /* Start lwiperf tcp server app. */
    /* Notice: the lwiperf just only support IPv4. */
    lwiperf_start_tcp_server_default(NULL, NULL);

    /* Force to set link up. */
    netif_set_link_up(&_netif);

    while (1)
    {
        ProcessEMACRx(&_netif);
    }
}
