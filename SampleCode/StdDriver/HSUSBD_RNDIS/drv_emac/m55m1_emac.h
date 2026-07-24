/**************************************************************************//**
 * @file     emac.h
 * @version  V1.00
 * @brief    M55M1 EMAC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __EMAC_H__
#define  __EMAC_H__

#include "NuMicro.h"
#include "synopGMAC_network_interface.h"

#define EMAC_RX_DESC_SIZE  RECEIVE_DESC_SIZE
#define EMAC_TX_DESC_SIZE  TRANSMIT_DESC_SIZE

void EMAC_Open(uint8_t *macaddr);
uint32_t EMAC_ReceivePkt(void);
int32_t  EMAC_TransmitPkt(uint8_t *pbuf, uint32_t len);
uint8_t *EMAC_AllocatePktBuf(void);
uint32_t EMAC_CheckLinkStatus(void);

#endif  /* __EMAC_H__ */
