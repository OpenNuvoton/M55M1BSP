/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * @brief    I2S driver sample configuration header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        512
#define BUFF_HALF_LEN   (BUFF_LEN/2)

/* Use LIN as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

extern void PDMA_ResetTxSGTable(uint8_t id);
extern void PDMA_ResetRxSGTable(uint8_t id);
extern void PDMA_Init(void);
#endif
