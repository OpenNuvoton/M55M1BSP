/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to set USCI_I2C use Multi bytes API Read and Write data to Slave.
 *          This sample code needs to work with USCI_I2C_Slave.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;

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
    /* Enable USCI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);
    /* Set multi-function pins for UI2C0 SDA and SCL */
    SET_USCI0_CLK_PA11();
    SET_USCI0_DAT0_PA10();
    /* USCI_I2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPIOA_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT10 | BIT11));
    /* Lock protected registers */
    SYS_LockReg();
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);
    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));
    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address : 0x4 */
}

void UI2C0_Close(void)
{
    /* Disable UI2C0 and close USCI clock */
    UI2C_Close(UI2C0);
    CLK_DisableModuleClock(USCI0_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;
    uint8_t err;
    uint8_t txbuf[256] = {0}, rDataBuf[256] = {0};
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code sets UI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+---------------------------------------------------------+\n");
    printf("| UI2C Driver Sample Code for Multi Bytes Read/Write Test |\n");
    printf("| Needs to work with USCI_I2C_Slave sample code           |\n");
    printf("|                                                         |\n");
    printf("|      UI2C Master (UI2C0) <---> UI2C Slave (UI2C0)       |\n");
    printf("| !! This sample code requires two boards to test !!      |\n");
    printf("+---------------------------------------------------------+\n");
    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection to UI2C0:\n");
    printf("UI2C0_SDA(PA10), UI2C0_SCL(PA11)\n");
    /* Init USCI_I2C0 */
    UI2C0_Init(100000);
    printf("Press any key to continue\n");
    getchar();
    /* Slave Address */
    g_u8DeviceAddr = 0x15;
    err = 0;

    /* Prepare data for transmission */
    for (i = 0; i < 256; i++)
    {
        txbuf[i] = (uint8_t) i + 3;
    }

    for (i = 0; i < 256; i += 32)
    {
        /* Write 32 bytes data to Slave */
        while (UI2C_WriteMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, i, &txbuf[i], 32) < 32);

        printf(".");
    }

    printf("Multi bytes Write access Pass.....\n");
    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while (UI2C_ReadMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, 0x0000, rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for (i = 0; i < 256; i++)
    {
        if (txbuf[i] != rDataBuf[i])
        {
            err = 1;
            printf("Data compare fail... R[%d] Data: 0x%X\n", i, rDataBuf[i]);
        }
    }

    if (err)
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
