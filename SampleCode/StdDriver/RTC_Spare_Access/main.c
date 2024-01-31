/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC spare_register read/write function and displays test result to the UART console.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5)*/
    SET_X32_IN_PF5();
    SET_X32_OUT_PF4();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Enable External LXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(RTC0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime;
    S_RTC_TIME_DATA_T *sptrInitTime;
    uint32_t i, u32SPRData = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    RTC Spare Register Read/Write Sample Code   |\n");
    printf("+------------------------------------------------+\n\n");


    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Time Setting */
    sInitTime.u32Year       = 2023;
    sInitTime.u32Month      = 6;
    sInitTime.u32Day        = 8;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 0;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_MONDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    /* check rtc reset status */
    sptrInitTime = (RTC->INIT & RTC_INIT_ACTIVE_Msk) ? NULL : &sInitTime;

    if (RTC_Open(sptrInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        return -1;
    }

    printf("# RTC Spare Register Read/Write Test:\n\n");

    /* Enable spare register */
    RTC_EnableSpareAccess();

    /* Write spare register */
    for (i = 0; i < 20; i++)
    {
        RTC_WRITE_SPARE_REGISTER(RTC, i, i);
    }

    /* Check spare register data */
    for (i = 0; i < 20; i++)
    {
        u32SPRData = RTC_READ_SPARE_REGISTER(RTC, i);

        if (u32SPRData != i)
        {
            printf(" SPARE_REGISTER[%u] = 0x%x.\n", i, u32SPRData);
            printf(" Get spare register Fail!! \n");
            return -1;
        }
        else
        {
            printf(" SPARE_REGISTER[%u] = 0x%x.\n", i, u32SPRData);
        }
    }

    printf("\n Compare spare registers data ... Pass!! \n");

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

