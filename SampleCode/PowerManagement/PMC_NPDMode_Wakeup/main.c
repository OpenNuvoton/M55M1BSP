/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system from NPD0-NPD4 Power-down mode.
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
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Check if all the debug messages are finished */
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
    /* Select the Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Enable the Brown-out Detector function */
    SYS_ENABLE_BOD();

    /* Enable the Brown-out Detector for power drop wake-up function. */
    SYS_SET_BOD_WAKEUP(SYS_BODCTL_BODWKEN_DROP);

    /* Set the Brown-out Detector voltage level to 3.0V */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_3_0V);

    /* Enable the Brown-out Detector interrupt function */
    SYS_DISABLE_BOD_RST();

    /* Clear PMC interrupt flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

    /* Enable PMC wake-up interrupt */
    PMC_ENABLE_WKINT();

    /* Enable Brown-out detector and Power-down wake-up interrupt */
    NVIC_EnableIRQ(BODOUT_IRQn);
    NVIC_EnableIRQ(PMC_IRQn);

    /* SCLK is invalid in power down mode, The de-glitch time must be change to LIRC before system enters power down mode */
    SYS_SET_BODDGSEL(SYS_BODCTL_BODDGSEL_LIRC);

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

    /* Enable PLL0 220MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

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
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void BODOUT_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1;

    /* Clear BOD Interrupt Flag */
    SYS_CLEAR_BOD_INT_FLAG();

    printf("Brown Out is Detected.\n");

    /* Wait BOD interrupt flag clear */
    while (SYS->BODSTS & SYS_BODSTS_BODIF_Msk)
    {
        if (--u32TimeOutCnt == 0) break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  PMC IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void PMC_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1;

    printf("Wake-up!!!\n");

    /* Clear PMC interrupt flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

    /* Wait PMC interrupt flag clear */
    while (PMC->INTSTS)
    {
        if (--u32TimeOutCnt == 0) break;
    }
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
        printf("|[3] NPD2 Wake-up by BOD Interrupt.                               |\n");
        printf("|[4] NPD3 Wake-up by BOD Interrupt.                               |\n");
        printf("|[5] NPD4 Wake-up by BOD Interrupt.                               |\n");
        printf("+-----------------------------------------------------------------+\n");
        u8Item = (uint8_t)getchar();

        /* Unlock protected registers */
        SYS_UnlockReg();

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

            default:
                break;
        }

        /* Lock protected registers */
        SYS_LockReg();
    }
}
