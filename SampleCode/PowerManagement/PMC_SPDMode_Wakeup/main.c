/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system form SPD Power-down mode by different wakeup sources.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void PowerDownFunction(void);
void WakeUpPinFunction(uint32_t u32PDMode);
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval);
void WakeUpLVRFunction(uint32_t u32PDMode);
void WakeUpBODFunction(uint32_t u32PDMode);
void CheckPowerSource(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    PMC_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up pin                         */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Configure GPIO as input mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);

    /* Set Wake-up pin trigger type at Deep Power down mode */
    PMC_EnableTGPin(PMC_TGPIN_PC, 0, PMC_TGPIN_FALLING, PMC_TGPIN_DEBOUNCEDIS, PMC_TGPIN_WAKEUP_ENABLE);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up Timer                         */
/*-----------------------------------------------------------------------------------------------------------*/
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{

    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Enable wake-up timer and set wake-up Time-out Interval */
    PMC_EnableSTMR(u32Interval);

    /* Enter to Power-down mode and wait for wake-up reset */
    PowerDownFunction();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by LVR                                 */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpLVRFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by BOD                                 */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpBODFunction(uint32_t u32PDMode)
{
    printf("Enter to SPD Power-down mode......\n");

    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Enable Brown-out detector function */
    SYS_ENABLE_BOD();

    /* Set Brown-out detector voltage level as 3.0V */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_3_0V);

    /* Enable Brown-out detector reset function */
    SYS_ENABLE_BOD_RST();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    uint32_t u32RegRstsrc1, u32RegRstsrc2;

    u32RegRstsrc1 = PMC_GetPMCWKSrc();
    u32RegRstsrc2 = SYS->RSTSTS;

    if ((u32RegRstsrc1 & PMC_INTSTS_GPCTGWKIF_Msk) != 0)
        printf("Wake-up source is TG Pin.\n");

    if ((u32RegRstsrc1 & PMC_INTSTS_STMRWKIF_Msk) != 0)
        printf("Wake-up source is Standby Wake-up Timer.\n");

    if ((u32RegRstsrc2 & SYS_RSTSTS_BODRF_Msk) != 0)
        printf("Wake-up source is BOD.\n");

    if ((u32RegRstsrc2 & SYS_RSTSTS_LVRRF_Msk) != 0)
        printf("Wake-up source is LVR.\n");

    /* Clear all wake-up flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

    /* Clear all wake-up flag */
    SYS->RSTSTS = SYS->RSTSTS;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    PMC_RELEASE_GPIO();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Item;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Get power manager wake up source */
    CheckPowerSource();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_SYSCLK, 3, CLK_CLKOCTL_DIV1EN_DIV_FREQSEL);

    printf("+-----------------------------------------------------------------+\n");
    printf("|    SPD Power-down Mode and Wake-up Sample Code                  |\n");
    printf("|    Please Select Power Down Mode and Wake up source.            |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("|[1] SPD TG pin(PC.0) and using rising edge wake up.              |\n");
    printf("|[2] SPD Wake-up TIMER time-out interval is 4096 LIRC clocks.     |\n");
    printf("|[3] SPD Wake-up by BOD.                                          |\n");
    printf("|[4] SPD Wake-up by LVR.                                          |\n");
    printf("+-----------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Disable wake-up timer */
    PMC_DisableSTMR();

    switch (u8Item)
    {
        case '1':
            WakeUpPinFunction(PMC_SPD0);
            break;

        case '2':
            WakeUpTimerFunction(PMC_SPD0, PMC_STMRWKCTL_STMRIS_4096);
            break;

        case '3':
            WakeUpBODFunction(PMC_SPD0);
            break;

        case '4':
            WakeUpLVRFunction(PMC_SPD0);
            break;

        default:
            break;
    }

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}
