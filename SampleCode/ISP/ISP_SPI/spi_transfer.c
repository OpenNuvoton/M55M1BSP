/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    ISP tool SPI initialization and IRQ function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "isp_user.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define TEST_COUNT  (16)
#define SPI_PORT    (SPI2)

uint32_t spi_rcvbuf[TEST_COUNT];
static volatile uint32_t s_u32TxDataCount;
static volatile uint32_t s_u32RxDataCount;

volatile uint8_t bSpiDataReady = 0;

void SPI_Init(void)
{
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI_PORT as a low level active device. */
    /* Disable I2S mode */
    SPI_PORT->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
    /* Default setting: slave selection signal is low level active. */
    SPI_PORT->SSCTL = SPI_SS_ACTIVE_LOW;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    SPI_PORT->CTL = SPI_SLAVE | ((32 & 0x1F) << SPI_CTL_DWIDTH_Pos) | (SPI_MODE_0) | SPI_CTL_SPIEN_Msk;
    /* Set DIVIDER = 0 */
    SPI_PORT->CLKDIV = 0UL;
    /* Set TX FIFO threshold and enable FIFO mode. */
    SPI_PORT->FIFOCTL = (SPI_PORT->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                        (4 << SPI_FIFOCTL_TXTH_Pos) |
                        (4 << SPI_FIFOCTL_RXTH_Pos);
    /* Enable slave selection signal active interrupt flag */
    SPI_PORT->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
    SPI_WRITE_TX(SPI_PORT, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */
    NVIC_EnableIRQ(SPI2_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  SPI_PORT IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void SPI2_IRQHandler(void)
{
    uint32_t *pu32ResponseBuff;
    uint32_t u32Data;

    pu32ResponseBuff = (uint32_t *)g_au8ResponseBuff; // in isp_user.c

    if (SPI_PORT->STATUS & SPI_STATUS_SSACTIF_Msk)
    {
        SPI_PORT->STATUS |= SPI_STATUS_SSACTIF_Msk;
        SPI_PORT->FIFOCTL |= (SPI_FIFOCTL_RXFBCLR_Msk | SPI_FIFOCTL_TXFBCLR_Msk);
        s_u32TxDataCount = 0;
        s_u32RxDataCount = 0;

        // Active
        while (!(SPI_PORT->STATUS & SPI_STATUS_SSINAIF_Msk))
        {
            /* Check TX FULL flag and TX data count */
            if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI_PORT) == 0) && (s_u32TxDataCount < TEST_COUNT))
            {
                SPI_WRITE_TX(SPI_PORT, pu32ResponseBuff[s_u32TxDataCount]);    /* Write to TX FIFO */
                s_u32TxDataCount++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL  = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            /* Check RX EMPTY flag */
            if (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_PORT) == 0)
            {
                u32Data = SPI_READ_RX(SPI_PORT);    /* Read RX FIFO */
                s_u32RxDataCount &= 0x0F;
                spi_rcvbuf[s_u32RxDataCount++] = u32Data;
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if (SPI_PORT->STATUS & SPI_STATUS_SSINAIF_Msk)
        {
            SPI_PORT->STATUS |= SPI_STATUS_SSINAIF_Msk;

            if ((s_u32RxDataCount == 16) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
            {
                bSpiDataReady = 1;
            }

            spi_rcvbuf[0] &= 0x000000FF;
            s_u32TxDataCount = 0;
            s_u32RxDataCount = 0;

            if (SPI_GET_TX_FIFO_FULL_FLAG(SPI_PORT) == 0)
            {
                SPI_WRITE_TX(SPI_PORT, 0xFFFFFFFF);    /* Write to TX FIFO */
            }
        }
    }
    else
    {
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/