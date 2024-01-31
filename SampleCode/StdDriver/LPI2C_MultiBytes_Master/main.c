/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief
 *          Show how to set LPI2C use Multi bytes API Read and Write data to Slave.
 *          Needs to work with LPI2C_Slave sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);
volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;

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
    /* Enable LPI2C0 module clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);
    /* Set multi-function pins for LPI2C0 SDA and SCL */
    SET_LPI2C0_SDA_PB4();
    SET_LPI2C0_SCL_PB5();
    /* LPI2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPIOB_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, (BIT4 | BIT5));
    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C0_Init(void)
{
    /* Open LPI2C0 and set clock to 100k */
    LPI2C_Open(LPI2C0, 100000);
    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;
    uint8_t u8Err;
    uint8_t txbuf[256] = {0}, rDataBuf[256] = {0};
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, Master accesses Slave with Multi Bytes Write
        and Multi Bytes Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+----------------------------------------------------------+\n");
    printf("| LPI2C Driver Sample Code for Multi Bytes Read/Write Test |\n");
    printf("| Needs to work with LPI2C_Slave sample code               |\n");
    printf("|                                                          |\n");
    printf("| LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)          |\n");
    printf("| !! This sample code requires two borads to test !!       |\n");
    printf("+----------------------------------------------------------+\n");
    printf("\n");
    /* Init LPI2C0 */
    LPI2C0_Init();
    /* Slave address */
    g_u8DeviceAddr = 0x15;
    u8Err = 0;

    /* Prepare data for transmission */
    for (i = 0; i < 256; i++)
    {
        txbuf[i] = (uint8_t) i + 3;
    }

    for (i = 0; i < 256; i += 32)
    {
        /* Write 32 bytes data to Slave */
        while (LPI2C_WriteMultiBytesTwoRegs(LPI2C0, g_u8DeviceAddr, i, &txbuf[i], 32) < 32);
    }

    printf("Multi bytes Write access Pass.....\n");
    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while (LPI2C_ReadMultiBytesTwoRegs(LPI2C0, g_u8DeviceAddr, 0x0000, rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for (i = 0; i < 256; i++)
    {
        if (txbuf[i] != rDataBuf[i])
        {
            u8Err = 1;
            printf("Data compare fail... R[%d] Data: 0x%X\n", i, rDataBuf[i]);
        }
    }

    if (u8Err)
    {
        printf("Multi bytes Read access Fail.....\n");
    }
    else
    {
        printf("Multi bytes Read access Pass.....\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
