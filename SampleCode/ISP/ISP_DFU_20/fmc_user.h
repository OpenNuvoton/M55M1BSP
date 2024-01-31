/***************************************************************************//**
 * @file     fmc_user.h
 * @version  V1.00
 * @brief    Simplified FMC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef FMC_USER_H
#define FMC_USER_H

#include "targetdev.h"

#define Config0         FMC_USER_CONFIG_0
#define Config1         FMC_USER_CONFIG_1
#define FMC_CONFIG_CNT  14

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


int FMC_Read_User(uint32_t u32Addr, uint32_t *data);
int FMC_Erase_User(uint32_t u32Addr);
void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
int EraseAP(uint32_t addr_start, uint32_t size);
void UpdateConfig(uint32_t *data, uint32_t *res);


#endif  /* FMC_USER_H */
