/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use I3C0 Master to receive and transmit the data through PDMA to a Slave.
 *          This sample code needs to work with I3C_SlaveRW.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#if (NVT_DCACHE_ON == 1)
    /* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
    uint32_t g_RxBuf[DCACHE_ALIGN_LINE_SIZE(I3C_DEVICE_RX_BUF_CNT * 4) / 4] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint32_t g_TxBuf[DCACHE_ALIGN_LINE_SIZE(I3C_DEVICE_TX_BUF_CNT * 4) / 4] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    __attribute__((aligned)) static uint32_t g_RxBuf[I3C_DEVICE_RX_BUF_CNT];
    __attribute__((aligned)) static uint32_t g_TxBuf[I3C_DEVICE_TX_BUF_CNT];
#endif

int32_t I3C_EnableRxPDMA(I3C_T *i3c, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts);
int32_t I3C_EnableTxPDMA(I3C_T *i3c, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts);

/**
  * @brief  Enable I3C Rx DMA function on PDMA CH-0.
  */
int32_t I3C_EnableRxPDMA(I3C_T *i3c, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts)
{
    uint8_t ch = 0;
    PDMA_RESET(PDMA0, ch);
    i3c->RSTCTL = (I3C_RESET_CMD_QUEUE | I3C_RESET_TX_BUF | I3C_RESET_RX_BUF);

    while (i3c->RSTCTL != 0) {};

    /* PDMA CH0 for I3C Rx */
    PDMA0->DSCT[ch].CTL =
        PDMA_OP_BASIC | PDMA_REQ_SINGLE |
        PDMA_SAR_FIX  | PDMA_DAR_INC |
        PDMA_WIDTH_32 |
        ((((u32ByteCnts + 3) / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos);

    PDMA0->DSCT[ch].SA = u32Src;

    PDMA0->DSCT[ch].DA = u32Dest;

    PDMA0->REQSEL0_3 &= ~PDMA_REQSEL0_3_REQSRC0_Msk;

    PDMA0->REQSEL0_3 |= (PDMA_I3C0_RX << PDMA_REQSEL0_3_REQSRC0_Pos);

    PDMA0->CHCTL |= (1 << ch);

    /* Enable I3C DMA function */
    I3C_EnableDMA(i3c);

    return 0;
}

/**
  * @brief  Enable I3C Tx DMA function on PDMA CH-1.
  */
int32_t I3C_EnableTxPDMA(I3C_T *i3c, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts)
{
    uint8_t ch = 1;
    PDMA_RESET(PDMA0, ch);
    i3c->RSTCTL = (I3C_RESET_CMD_QUEUE | I3C_RESET_TX_BUF | I3C_RESET_RX_BUF);

    while (i3c->RSTCTL != 0) {};

    /* PDMA CH1 for I3C Tx */
    PDMA0->DSCT[ch].CTL =
        PDMA_OP_BASIC | PDMA_REQ_SINGLE |
        PDMA_SAR_INC  | PDMA_DAR_FIX |
        PDMA_WIDTH_32 |
        ((((u32ByteCnts + 3) / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos);

    PDMA0->DSCT[ch].SA = u32Src;

    PDMA0->DSCT[ch].DA = u32Dest;

    PDMA0->REQSEL0_3 &= ~PDMA_REQSEL0_3_REQSRC1_Msk;

    PDMA0->REQSEL0_3 |= (PDMA_I3C0_TX << PDMA_REQSEL0_3_REQSRC1_Pos);

    PDMA0->CHCTL |= (1 << ch);

    /* Enable I3C DMA function */
    I3C_EnableDMA(i3c);

    return 0;
}


/**
  * @brief      Write data to Slave through PDMA
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u32Speed        the speed in which the transfer should be driven. It could be
  *                                 \ref I3C_DEVI3C_SPEED_SDR0
  *                                 \ref I3C_DEVI3C_SPEED_SDR1
  *                                 \ref I3C_DEVI3C_SPEED_SDR2
  *                                 \ref I3C_DEVI3C_SPEED_SDR3
  *                                 \ref I3C_DEVI3C_SPEED_SDR4
  *                                 \ref I3C_DEVI3C_SPEED_HDRDDR
  *                                 \ref I3C_DEVI3C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFMPLUS
  * @param[in]  *pu32TxBuf      Pointer to array to write data to Slave
  * @param[in]  u16WriteBytes   How many bytes need to write to Slave
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3C_STS_TX_FULL         TX FIFO is full
  *
  * @details    The function is used for I3C Master write data to Slave.
  *
  * @note       Device Address Table must be set before using this function.
  *
  */
int32_t I3C_WritePDMA(I3C_T *i3c, uint8_t u8DevIndex, uint32_t u32Speed, uint32_t *pu32TxBuf, uint16_t u16WriteBytes)
{
    uint32_t response;

    if ((u16WriteBytes == 0) || (pu32TxBuf == NULL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    {
        i3c->CMDQUE = ((u16WriteBytes << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);

        /* Check if CmdQ is full */
        if (I3C_IS_CMDQ_FULL(i3c))
        {
            return I3C_STS_CMDQ_FULL;
        }

        i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                       | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                       | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                       | (I3C_TX_TID << I3C_CMDQUE_TID_Pos)
                       | I3C_CMDATTR_TRANSFER_CMD);
        /* Enable I3C PDMA transmit function */
        I3C_EnableTxPDMA(i3c, (uint32_t)(pu32TxBuf), (uint32_t)(&i3c->TXRXDAT), u16WriteBytes);
    }

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    I3C_DisableDMA(i3c);
    response = i3c->RESPQUE;

    if (response & I3C_RESPQUE_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Read data from Slave through PDMA
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u32Speed        the speed in which the transfer should be driven. It could be
  *                                 \ref I3C_DEVI3C_SPEED_SDR0
  *                                 \ref I3C_DEVI3C_SPEED_SDR1
  *                                 \ref I3C_DEVI3C_SPEED_SDR2
  *                                 \ref I3C_DEVI3C_SPEED_SDR3
  *                                 \ref I3C_DEVI3C_SPEED_SDR4
  *                                 \ref I3C_DEVI3C_SPEED_HDRDDR
  *                                 \ref I3C_DEVI3C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFMPLUS
  * @param[in]  *pu32RxBuf      Pointer to array to read data from Slave
  * @param[in]  u16ReadBytes    How many bytes need to read from Slave
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  *
  * @details    The function is used for I3C Master write data to Slave.
  *
  * @note       Device Address Table must be set before using this function.
  * @note       if u16ReadBytes is not
  *
  */
int32_t I3C_ReadPDMA(I3C_T *i3c, uint8_t u8DevIndex, uint32_t u32Speed, uint32_t *pu32RxBuf, uint16_t u16ReadBytes)
{
    uint32_t u32TimeOutCount = (SystemCoreClock / 1000);
    uint32_t response;

    if ((u16ReadBytes == 0) || (pu32RxBuf == NULL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    i3c->CMDQUE = ((u16ReadBytes << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_RNW_Msk | I3C_CMDQUE_ROC_Msk
                   | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                   | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                   | (I3C_RX_TID << I3C_CMDQUE_TID_Pos)
                   | I3C_CMDATTR_TRANSFER_CMD);
    /* Enable I3C PDMA receive function */
    I3C_EnableRxPDMA(i3c, (uint32_t)(&i3c->TXRXDAT), (uint32_t)(pu32RxBuf), (I3C_DEVICE_RX_BUF_CNT * 4));

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0)
    {
        if (--u32TimeOutCount == 0)
        {
            return I3C_TIMEOUT_ERR;
        }
    };

    I3C_DisableDMA(i3c);

    response = i3c->RESPQUE;

    if ((response & I3C_RESPQUE_ERRSTS_Msk) != I3C_RESP_NO_ERR)
    {
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set SCL slew rate to GPIO_SLEWCTL_FAST0, SDA slew rate to GPIO_SLEWCTL_HIGH */
    GPIO_SetSlewCtl(PB, BIT1, GPIO_SLEWCTL_FAST0);
    GPIO_SetSlewCtl(PB, BIT0, GPIO_SLEWCTL_HIGH);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint16_t    i;
    uint8_t     *pu8Data;
    int32_t     ret;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------------+\n");
    printf("|    I3C0 Master Read/Write through PDMA Sample Code   |\n");
    printf("+------------------------------------------------------+\n\n");
    /* Initial I3C0 default settings */
    I3C_Open(I3C0, I3C_MASTER, 0, 0);
    /* Enable I3C0 controller */
    I3C_Enable(I3C0);
    /* Dynamic Address for Enter Dynamic Address Assignment (ENTDAA) */
    I3C_SetDeviceAddr(I3C0, 0, I3C_DEVTYPE_I3C, 0x18, 0x00);

    while (1)
    {
        printf("press any key to broadcast ENTDAA\n");
        getchar();

        if (1 == I3C_BroadcastENTDAA(I3C0, 1))
        {
            printf("I3C Device found:\n");
            printf(" - Provisional ID = 0x%08X%02X\n", I3C0->DEV1CH[0], I3C0->DEV1CH[1]);
            printf(" - BCR, DCR = 0x%08X\n", I3C0->DEV1CH[2]);
            printf(" - DADR = 0x%08X\n\n", I3C0->DEV1CH[3]);
            break;
        }
    }

    pu8Data = (uint8_t *)g_TxBuf;

    for (i = 0; i < 16; i++)
    {
        pu8Data[i] = i;
    }

    while (1)
    {
#if (NVT_DCACHE_ON == 1)
        /*
            Clean the CPU Data cache before starting the DMA transfer.
            This guarantees that the source buffer will be up to date before starting the transfer.
        */
        SCB_CleanDCache_by_Addr(g_TxBuf, sizeof(g_TxBuf));
#endif  // (NVT_DCACHE_ON == 1)
        printf("press any key to Write I3C Target \n");
        getchar();
        ret = I3C_WritePDMA(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)g_TxBuf, 16);

        if (ret != I3C_STS_NO_ERR)
        {
            printf("I3C_Write Fail (%d)\n", ret);

            while (1);
        }

        printf("press any key to Read I3C Target \n");
        getchar();
        ret = I3C_ReadPDMA(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)g_RxBuf, 16);

        if (ret != I3C_STS_NO_ERR)
        {
            printf("I3C_Read Fail (%d)\n", ret);

            while (1);
        }

#if (NVT_DCACHE_ON == 1)
        /*
            Invalidate the CPU Data cache after the DMA transfer.
            As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
        */
        SCB_InvalidateDCache_by_Addr(g_RxBuf, sizeof(g_RxBuf));
#endif  // (NVT_DCACHE_ON == 1)

        for (i = 0; i < 4; i++)
        {
            if (g_RxBuf[i] != g_TxBuf[i])
            {
                printf("Compare Data Fail\n");

                while (1);
            }
        }

        printf("Compare Data Pass\n\n");
        pu8Data[0] += 1;
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
