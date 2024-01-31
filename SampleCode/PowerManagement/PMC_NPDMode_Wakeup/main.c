/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system form NPD Power-down mode by different wakeup sources.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void PowerDownFunction(void);
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
/*  Function for System Entry to Power Down Mode and Wake up source by BOD                                 */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpBODFunction(uint32_t u32PDMode)
{
    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Enable Brown-out detector function */
    SYS_ENABLE_BOD();

    /* Set Brown-out detector voltage level to 3.0V */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_3_0V);

    /* Enable Brown-out detector reset function */
    SYS_DISABLE_BOD_RST();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
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

NVT_ITCM void PMC_IRQHandler(void)
{
    printf("Wake-up!!!\n");

    /* Clear PMC interrupt flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    inp32(&PMC->INTSTS);
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

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_SYSCLK, 3, CLK_CLKOCTL_DIV1EN_DIV_FREQSEL);

    while (1)
    {
        printf("+-----------------------------------------------------------------+\n");
        printf("|    NPD Power-down Mode and Wake-up Sample Code                  |\n");
        printf("|    Please Select Power Down Mode                                |\n");
        printf("+-----------------------------------------------------------------+\n");
        printf("|[1] NPD0 Wake-up by BOD Interrupt.                               |\n");
        printf("|[2] NPD1 Wake-up by BOD Interrupt.                               |\n");
#if 0   // TESTCHIP_ONLY not support        
        printf("|[3] NPD2 Wake-up by BOD Interrupt.                               |\n");
        printf("|[4] NPD3 Wake-up by BOD Interrupt.                               |\n");
        printf("|[5] NPD4 Wake-up by BOD Interrupt.                               |\n");
#endif
        printf("+-----------------------------------------------------------------+\n");
        u8Item = (uint8_t)getchar();

        /* Unlock protected registers */
        SYS_UnlockReg();

        /* Enable PMC Interrupt */
        PMC_ENABLE_INT();
        NVIC_EnableIRQ(PMC_IRQn);

        /* Disable wake-up timer */
        PMC_DisableSTMR();

        switch (u8Item)
        {
            case '1':
                printf("Enter to NPD0 Power-down mode......\n");
                WakeUpBODFunction(PMC_NPD0);
                break;

            case '2':
                printf("Enter to NPD1 Power-down mode......\n");
                WakeUpBODFunction(PMC_NPD1);
                break;
#if 0   // TESTCHIP_ONLY not support

            case '3':
                printf("Enter to NPD2 Power-down mode......\n");
                WakeUpBODFunction(PMC_NPD2);
                break;

            case '4':
                printf("Enter to NPD3 Power-down mode......\n");
                WakeUpBODFunction(PMC_NPD3);
                break;

            case '5':
                printf("Enter to NPD4 Power-down mode......\n");
                WakeUpBODFunction(PMC_NPD4);
                break;
#endif

            default:
                break;
        }

        /* Lock protected registers */
        SYS_LockReg();
    }
}
