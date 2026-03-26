/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrates how to initialize the I3C Controller and perform operations to the I3C Target.
 *           This sample code needs to use two boards.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "i3c_DeviceFunc.h"

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
    /* Enable PLL0 200MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_200MHZ);
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
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set SCL slew rate to GPIO_SLEWCTL_FAST0, SDA slew rate to GPIO_SLEWCTL_HIGH */
    GPIO_SetSlewCtl(PB, BIT1, GPIO_SLEWCTL_FAST0);
    GPIO_SetSlewCtl(PB, BIT0, GPIO_SLEWCTL_FAST0);
    /* I3C pins enable schmitt trigger */
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, BIT0 | BIT1);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("System core clock: %d\n", SystemCoreClock);
    printf("+----------------------------------+\n");
    printf("|    I3C Controller Sample Code    |\n");
    printf("+----------------------------------+\n\n");
    printf("# I3C Controller Settings:\n");
    printf("    - SDA on PB.0\n");
    printf("    - SCL on PB.1\n");
    printf("    - PDMA %s\n", g_I3CDev.is_DMA ? "enabled" : "disabled");
    printf("# Supports:\n");
    printf("    - I2C FM+ Write/Read to I2C Target\n");
    printf("    - I3C Dynamic Address Assignment through ENTDAA CCC for I3C Targets\n");
    printf("        - Reset all Targets to I2C mode RSTDAA CCC\n");
    printf("    - SDR and HDR-DDR Write/Read to I3C Target\n");
    printf("    - Common Command Codes\n");
    printf("    - Response In-Band Interrupt Request from Target\n");
    printf("    - Response Controller Request Request from Target\n");
    printf("# User can hit [Enter] to display the function menu.\n");
    printf("\n");
    printf("[ User needs to press [e] to send ENTDAA CCC for I3C Target Dynamic Address Assignment ]\n\n");
    /* Initialize the device as I3C Controller Role */
    I3C_ControllerRole(&g_I3CDev, 1);

    while (1)
    {
        /* Device has switched to I3C Target Role */
        I3C_TargetRole(&g_I3CDev, 0);
        /* Device has switched to I3C Controller Role */
        I3C_ControllerRole(&g_I3CDev, 0);
    }
}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
