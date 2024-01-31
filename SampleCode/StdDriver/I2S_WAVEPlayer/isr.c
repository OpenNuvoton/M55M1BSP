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
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & 0x2)   /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & 0x4)
        {
            g_u8PCMBufferFull[g_u8PCMBufferPlaying] = 0;       /* Set empty flag */
            g_u8PCMBufferPlaying ^= 1;
        }

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else if (u32Status & 0x400)  /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
}
