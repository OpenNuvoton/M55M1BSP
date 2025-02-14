/**
 * @file
 *
 * lwIP Options Configuration
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include <stdlib.h>

/*
 * Include user defined options first. Anything not defined in these files
 * will be set to standard values. Override anything you dont like!
 */

#define NO_SYS                             1

#define LWIP_NOASSERT
#define MEM_SIZE                     (64*1024)
#define MEM_ALIGNMENT                      32
#define MEMP_NUM_PBUF                      64
#define MEMP_NUM_UDP_PCB                   32
#define MEMP_NUM_TCP_PCB                   32
#define MEMP_NUM_TCP_PCB_LISTEN             8
#define MEMP_NUM_TCP_SEG                    8
#define MEMP_NUM_SYS_TIMEOUT               10
#define PBUF_POOL_SIZE                      8
#define PBUF_POOL_BUFSIZE                1524
#define LWIP_TCP                            1
#define TCP_TTL                           255
#define LWIP_SO_RCVTIMEO                    1
#define LWIP_SO_RCVRCVTIMEO_NONSTANDARD     1
#define LWIP_SO_SNDTIMEO                    1
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD     1
#define TCP_QUEUE_OOSEQ                     0
#define TCP_MSS                   (1500 - 40)
#define TCP_SND_BUF               (4*TCP_MSS)
#define TCP_SND_QUEUELEN          (2* TCP_SND_BUF/TCP_MSS)
#define TCP_WND                   (3*TCP_MSS)
#define LWIP_TCP_KEEPALIVE                  1
#define LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS  1
#define LWIP_ICMP                           1
#define LWIP_RAW                            1
#define DEFAULT_RAW_RECVMBOX_SIZE           3
#define LWIP_DHCP                           1
#define LWIP_UDP                            1
#define UDP_TTL                           255
#define LWIP_STATS                          0
#define LWIP_NETIF_HOSTNAME                 1
#define LWIP_NETIF_STATUS_CALLBACK          1
#define LWIP_NETIF_LINK_CALLBACK            1
#define LWIP_DHCP_CHECK_LINK_UP             1
#define LWIP_NETCONN                        0
#define LWIP_SOCKET                         0
#define LWIP_SOCKET_SET_ERRNO               0
#define LWIP_USING_HW_CHECKSUM              1

#define LWIP_IPV6                           1
#define LWIP_IPV6_DHCP6                     1
#define LWIP_IPV6_MLD                       1
#define LWIP_RAND()         ((uint32_t)rand())

/* ---------- Checksum options ---------- */
#if (LWIP_USING_HW_CHECKSUM == 1)
    #define CHECKSUM_GEN_IP                 0
    #define CHECKSUM_GEN_UDP                0
    #define CHECKSUM_GEN_TCP                0
    #define CHECKSUM_GEN_ICMP               0
    #define CHECKSUM_CHECK_IP               0
    #define CHECKSUM_CHECK_UDP              0
    #define CHECKSUM_CHECK_TCP              0
    #define CHECKSUM_CHECK_ICMP             0
#else
    #define CHECKSUM_GEN_IP                 1
    #define CHECKSUM_GEN_UDP                1
    #define CHECKSUM_GEN_TCP                1
    #define CHECKSUM_GEN_ICMP               1
    #define CHECKSUM_CHECK_IP               1
    #define CHECKSUM_CHECK_UDP              1
    #define CHECKSUM_CHECK_TCP              1
    #define CHECKSUM_CHECK_ICMP             1
#endif

#endif /* __LWIPOPTS_H__ */
