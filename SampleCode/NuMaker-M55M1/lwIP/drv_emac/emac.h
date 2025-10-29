/**************************************************************************//**
 * @file     emac.h
 * @version  V1.00
 * @brief    EMAC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __EMAC_H__
#define  __EMAC_H__

#include "NuMicro.h"
#include "synopGMAC_network_interface.h"

void EMAC_Open(uint8_t *macaddr);
uint32_t EMAC_ReceivePkt(void);
int32_t  EMAC_TransmitPkt(uint8_t *pbuf, uint32_t len);
uint8_t *EMAC_AllocatePktBuf(void);

#endif  /* __EMAC_H__ */
