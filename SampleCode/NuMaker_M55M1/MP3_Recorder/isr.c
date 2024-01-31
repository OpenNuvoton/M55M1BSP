/*************************************************************************//**
 * @file     isr.c
 * @version  V3.00
 * @brief    ISR source file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "l3.h"

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
volatile uint8_t g_u8PCMBuffer_Playing = 0;

volatile uint32_t g_au32PcmBuff1[PCM_BUFFER_SIZE] = {0};
volatile uint32_t g_au32PcmBuff2[PCM_BUFFER_SIZE] = {0};

volatile uint32_t g_u32BuffPos = 0;
volatile uint32_t g_u32BuffPos1 = 0, g_u32BuffPos2 = 0;
volatile uint32_t g_u32BuffToggle = 0;

volatile uint32_t g_u32ErrorFlag = 0;

extern shine_config_t config;
extern int32_t        samples_per_pass;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & 0x2)   /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & 0x4)             /* channel 2 done */
        {
            if (g_au8PCMBuffer_Full[g_u8PCMBuffer_Playing ^ 1] != 1)
                printf("underflow!!\n");

            g_au8PCMBuffer_Full[g_u8PCMBuffer_Playing] = 0;       /* Set empty flag */
            g_u8PCMBuffer_Playing ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }
    }
    else if (u32Status & 0x400)    /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        printf("PDMA Timeout!!\n");
    }
    else
    {
        printf("0x%x\n", u32Status);

        while (1);
    }
}

void I2S0_IRQHandler(void)
{
    uint32_t u32Reg;
    uint32_t u32Len, i;
    int32_t *pi32BuffRx1, *pi32BuffRx2;

    u32Reg = I2S_GET_INT_FLAG(I2S0, I2S_STATUS0_RXTHIF_Msk);

    if (u32Reg & I2S_STATUS0_RXTHIF_Msk)
    {
        /* Read RX FIFO Level */
        u32Len = I2S_GET_RX_FIFO_LEVEL(I2S0);

#ifdef REC_IN_RT

        if ((g_u32BuffPos1 > 0) && (g_u32BuffPos1 == ((samples_per_pass * config.wave.channels) >> 1)))
        {
            if (g_u32WriteSDToggle != 0)
            {
                g_u32ErrorFlag++;
                g_u32BuffPos1 = g_u32BuffPos2 = 0;
            }

            g_u32BuffToggle = 1;
        }

        if ((g_u32BuffPos2 > 0) && (g_u32BuffPos2 == ((samples_per_pass * config.wave.channels) >> 1)))
        {
            if (g_u32WriteSDToggle != 1)
            {
                g_u32ErrorFlag++;
                g_u32BuffPos1 = g_u32BuffPos2 = 0;
            }

            g_u32BuffToggle = 0;
        }

        if (g_u32BuffToggle == 0)
        {
            pi32BuffRx1 = (int32_t *)&g_au32PcmBuff1[g_u32BuffPos1];

            for (i = 0; i < u32Len; i++)
            {
                pi32BuffRx1[i] = I2S_READ_RX_FIFO(I2S0);
            }

            g_u32BuffPos1 += u32Len;
        }
        else if (g_u32BuffToggle == 1)
        {
            pi32BuffRx2 = (int32_t *)&g_au32PcmBuff2[g_u32BuffPos2];

            for (i = 0; i < u32Len; i++)
            {
                pi32BuffRx2[i] = I2S_READ_RX_FIFO(I2S0);
            }

            g_u32BuffPos2 += u32Len;
        }

#else

        for (i = 0; i < u32Len; i++)
        {
            outp32(HYPER_RAM_MEM_MAP + g_u32BuffPos + 4 * i, I2S_READ_RX_FIFO(I2S0));
        }

        g_u32BuffPos += 4 * u32Len;

        if (g_u32BuffPos >= 0x700000)
        {
            /* Disable I2S RX function */
            I2S_DISABLE_RX(I2S0);
            g_u32RecordDone = 1;

            printf("The record data size reaches the allocated HyperRAM size of 7 Mbytes.\n");
        }

#endif
    }
}
