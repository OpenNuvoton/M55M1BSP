/******************************************************************************
 * @file    main.c
 * @version V1.00
 * @brief   Firmware update sample code (Bank0 App).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"

#define TEST_MODE    1

static volatile uint32_t s_u32ExecBank;
static volatile uint32_t s_u32GetSum;

void     SYS_Init(void);
int32_t  SelfTest(void);
uint32_t FuncCrc32(uint32_t u32Start, uint32_t u32Len);

uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for (u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }

    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

NVT_ITCM void WDT0_IRQHandler(void)
{
    WDT_RESET_COUNTER(WDT0);

    if (WDT_GET_TIMEOUT_INT_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(CRC0_MODULE);
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t  SelfTest(void)
{
#if TEST_MODE
    uint8_t u8GetCh;

    printf("\n Self test pass ? [y/n] \n");
    u8GetCh = (uint8_t)getchar();
    printf("\n User select [%c] \n", u8GetCh);

    /* Let user select to test pass or fail condition*/
    if (u8GetCh == 'y')
    {
        printf("\n Self test pass. \n\n");
        return 0;
    }
    else
    {
        printf("\n Self test fail ! \n\n");
        return -1;
    }

#else
    s_u32GetSum = FMC_GetChkSum(APP_BASE, APP_SIZE);
    g_u32KeepSum = FMC_Read(FW_CRC_BASE);
    printf("\n GetSum = 0x%x, Keep Sum = 0x%x \n", s_u32GetSum, g_u32KeepSum);

    if (s_u32GetSum == g_u32KeepSum)
    {
        printf("\n Self test pass. \n");
        return 0;
    }
    else
    {
        printf("\n Self test fail!!! \n");
        return -1;
    }

#endif
}


int main()
{
    uint32_t i;
    int32_t i32Ret;

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Init Debug UART for print message */
    InitDebugUart();

    /* Disable register write-protection function */
    SYS_UnlockReg();
    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    /* Unlock protected registers */
    SYS_UnlockReg();

    NVIC_EnableIRQ(WDT0_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT0, WDT_TIMEOUT_2POW18, WDT_RESET_DELAY_18CLK, 1, 0);

    /* Enable WDT interrupt function */
    WDT_EnableInt(WDT0);

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        /* Check CPU run at Bank0 or Bank1 */
        s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n Bank%d APP processing (New firmware)\n", s_u32ExecBank);

        /* Execute firmware self test */
        i32Ret = SelfTest();

        if (i32Ret == 0)
        {
            /* Normal test condition*/
            for (i = 0; i < 1000; i++)
            {
                printf(" Firmware processing ... Count [%d]\r", i);
                s_u32GetSum = FuncCrc32(APP_BASE, APP_SIZE);
            }
        }
        else
        {
            /* Failure test condition, will reset by WDT and start from Bank0 loader */
            printf("\n Enter power down...\n");
            CLK_SysTickDelay(2000);
            PMC_PowerDown();
        }
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
