/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief
 *          Show how to use LPI2C Single byte API Read and Write data to Slave
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
    uint8_t u8Data, u8Tmp, u8Err;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+----------------------------------------------------------+\n");
    printf("| LPI2C Driver Sample Code for Single Byte Read/Write Test |\n");
    printf("| Needs to work with LPI2C_Slave sample code               |\n");
    printf("|                                                          |\n");
    printf("| LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)          |\n");
    printf("| !! This sample code requires two boards to test !!       |\n");
    printf("+----------------------------------------------------------+\n");
    printf("\n");
    /* Init LPI2C0 */
    LPI2C0_Init();
    /* Slave Address */
    g_u8DeviceAddr = 0x15;
    u8Err = 0;

    for (i = 0; i < 256; i++)
    {
        u8Tmp = (uint8_t)i + 3;

        /* Single Byte Write (Two Registers) */
        while (LPI2C_WriteByteTwoRegs(LPI2C0, g_u8DeviceAddr, i, u8Tmp));

        /* Single Byte Read (Two Registers) */
        u8Data = LPI2C_ReadByteTwoRegs(LPI2C0, g_u8DeviceAddr, i);

        if (u8Data != u8Tmp)
        {
            u8Err = 1;
            printf("%03d: Single byte write data fail,  W(0x%X)/R(0x%X) \n", i, u8Tmp, u8Data);
        }
    }

    printf("\n");

    if (u8Err)
    {
        printf("Single byte Read/Write access Fail.....\n");
    }
    else
    {
        printf("Single byte Read/Write access Pass.....\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
