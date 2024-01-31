/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use I3C0 Master to transmit and receive the data from a Slave.
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
volatile uint32_t   g_RxBuf[I3C_DEVICE_RX_BUF_CNT], g_TxBuf[I3C_DEVICE_RX_BUF_CNT];

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);
    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);
    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
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
    uint32_t    u32Data;
    int32_t     ret;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    I3C0 Master Read/Write Sample Code  |\n");
    printf("+----------------------------------------+\n\n");
    /* Initial I3C0 default settings */
    I3C_Open(I3C0, I3C_MASTER, 0, 0);
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
        /* Single Byte Write & Read Test */
        u32Data = 0x55;
        printf("press any key to Write I3C Target \n");
        getchar();
        ret = I3C_Write(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)&u32Data, 1);

        if (ret != I3C_STS_NO_ERR)
        {
            printf("I3C_Write Fail\n");

            while (1);
        }

        printf("press any key to Read I3C Target \n");
        getchar();
        ret = I3C_Read(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)&u32Data, 1);

        if (ret != I3C_STS_NO_ERR)
        {
            printf("I3C_Read Fail\n");

            while (1);
        }

        /* Multiple Bytes Write & Read Test */
        printf("press any key to Write I3C Target \n");
        getchar();
        ret = I3C_Write(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)g_TxBuf, 16);

        if (ret != I3C_STS_NO_ERR)
        {
            printf("I3C_Write Fail\n");

            while (1);
        }

        printf("press any key to Read I3C Target \n");
        getchar();
        ret = I3C_Read(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)g_RxBuf, 16);

        if (ret != I3C_STS_NO_ERR)
        {
            printf("I3C_Read Fail\n");

            while (1);
        }

        for (i = 0; i < 4; i++)
        {
            if (g_RxBuf[i] != g_TxBuf[i])
            {
                printf("Compare Data Fail\n");
                printf("g_RxBuf = 0x%08X, 0x%08X, 0x%08X, 0x%08X\n", g_RxBuf[0], g_RxBuf[1], g_RxBuf[2], g_RxBuf[3]);
                printf("g_TxBuf = 0x%08X, 0x%08X, 0x%08X, 0x%08X\n", g_TxBuf[0], g_TxBuf[1], g_TxBuf[2], g_TxBuf[3]);

                while (1);
            }
        }

        printf("Compare Data Pass\n\n");
        pu8Data[0] += 1;
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
