/******************************************************************************
 * @file     isr.c
 * @version  V1.00
 * @brief    ISR source file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"

//------------------------------------------------------------------------------
static uint32_t g_au32PcmBuff[32] = {0};

//------------------------------------------------------------------------------
extern void I2S0_IRQHandler(void);

//------------------------------------------------------------------------------
NVT_ITCM void I2S0_IRQHandler(void)
{
    uint32_t u32Reg;
    uint32_t u32Len, u32i;
    uint32_t *pu32BuffTx, *pu32BuffRx;

    u32Reg = I2S_GET_INT_FLAG(I2S0, I2S_STATUS0_TXTHIF_Msk | I2S_STATUS0_RXTHIF_Msk);

    if (u32Reg & I2S_STATUS0_TXTHIF_Msk)
    {
        pu32BuffTx = &g_au32PcmBuff[0];

        /* Read Tx FIFO free size */
        u32Len = 8 - I2S_GET_TX_FIFO_LEVEL(I2S0);

        if (g_u32BuffPos >= 8)
        {
            for (u32i = 0; u32i < u32Len; u32i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, pu32BuffTx[u32i]);
            }

            for (u32i = 0; u32i < BUFF_LEN - u32Len; u32i++)
            {
                pu32BuffTx[u32i] = pu32BuffTx[u32i + u32Len];
            }

            g_u32BuffPos -= u32Len;
        }
        else
        {
            for (u32i = 0; u32i < u32Len; u32i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, 0x00);
            }
        }
    }

    if (u32Reg & I2S_STATUS0_RXTHIF_Msk)
    {
        if (g_u32BuffPos < (BUFF_LEN - 8))
        {
            pu32BuffRx = &g_au32PcmBuff[g_u32BuffPos];

            /* Read Rx FIFO Level */
            u32Len = I2S_GET_RX_FIFO_LEVEL(I2S0);

            for (u32i = 0; u32i < u32Len; u32i++)
            {
                pu32BuffRx[u32i] = I2S_READ_RX_FIFO(I2S0);
            }

            g_u32BuffPos += u32Len;

            if (g_u32BuffPos >= BUFF_LEN)
            {
                g_u32BuffPos =    0;
            }
        }
    }
}
