/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to minimize power consumption when entering Power-down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> NPD0
//      <1=> NPD1
//      <2=> NPD2
//      <3=> NPD3
//      <4=> NPD4
//      <5=> SPD0
//      <6=> SPD1
//      <7=> DPD
*/
#define SET_PDMSEL    0

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


#define GPIO_P0_TO_P15      0xFFFF


void PowerDownFunction(void);
void GPC_IRQHandler(void);
void POR_Setting(void);
int32_t LIRC_Setting(void);
int32_t LXT_Setting(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Select power-down mode */
    PMC_SetPowerDownMode(SET_PDMSEL, PMC_PLCTL_PLSEL_PL1);

    printf("Entering power-down mode...\n");

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Enter to power-down mode */
    PMC_PowerDown();
}

/**
 * @brief       GPIO PC IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PC default IRQ
 */
NVT_ITCM void GPC_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PC.0 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PC, BIT0))
    {
        GPIO_CLR_INT_FLAG(PC, BIT0);
        printf("PC.0 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        u32temp = PC->INTSRC;
        PC->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    inp32(PC->INTSRC);
}

void POR_Setting(void)
{
    if (SET_POR == 0)
    {
        /* Disable POR */
        SYS_DISABLE_POR();
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
    }
}

int32_t LIRC_Setting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK_DisableXtalRC(CLK_SRCCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

        if (CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LXT_Setting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LXT == 0)
    {
        /* Disable LXT and wait for LXT stable flag is cleared */
        CLK_DisableXtalRC(CLK_SRCCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LXT and wait for LXT stable flag is set */
        CLK_EnableXtalRC(CLK_SRCCTL_LXTEN_Msk);

        if (CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
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

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt, u32PMUSTS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    /* Clear SPD/DPD mode wake-up status for entering SPD/DPD mode again */
    u32PMUSTS = PMC->INTSTS;

    if (u32PMUSTS)
    {
        /* Release I/O hold status for SPD mode */
        PMC_RELEASE_GPIO();

        /* Clear SPD/DPD mode wake-up status */
        PMC->INTSTS = PMC_INTSTS_CLRWK_Msk;

        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (PMC->INTSTS)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for SPD/DPD mode wake-up status is cleared time-out!\n");
                goto lexit;
            }
        }
    }

    /* Check SPD/DPD mode PC.0 falling-edge wake-up event */
    if (u32PMUSTS & PMC_INTSTS_PIN0WKIF_Msk)
    {
        printf("System waken-up done.\n\n");

        while (1);
    }

    /* Set all multi-function pins to GPIO mode except UART and ICE pins */
    memset((void *)(&SYS->GPA_MFP0), 0x0, sizeof(SYS->GPA_MFP0) * 40);
    SET_ICE_CLK_PF1();
    SET_ICE_DAT_PF0();
    SetDebugUartMFP();

    /* Configure all GPIO as Quasi-bidirectional mode */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PI, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PJ, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PC.0 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("| 1. Remove all continuous load, e.g. LED.                          |\n");
    printf("| 2. Configure all GPIO as Quasi-bidirectional Mode                 |\n");
    printf("| 3. Disable analog function, e.g. POR module                       |\n");
    printf("| 4. Disable unused clock, e.g. LIRC                                |\n");
    printf("| 5. Enter to Power-Down                                            |\n");
    printf("| 6. Wait for PC.0 falling-edge interrupt event to wake-up the MCU  |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* POR setting */
    POR_Setting();

    /* LIRC setting */
    if (LIRC_Setting() < 0) goto lexit;

    /* LXT setting */
    if (LXT_Setting() < 0) goto lexit;

    /* Wake-up source configuration */
    if ((SET_PDMSEL == PMC_NPD0) || (SET_PDMSEL == PMC_NPD1) || (SET_PDMSEL == PMC_NPD2))
    {
        /* Configure PC.0 as Quasi mode and enable interrupt by falling edge trigger */
        CLK_EnableModuleClock(GPIOC_MODULE);
        GPIO_SetMode(PC, BIT0, GPIO_MODE_QUASI);
        GPIO_EnableInt(PC, 0, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPC_IRQn);
    }
    else if ((SET_PDMSEL == PMC_NPD3) || (SET_PDMSEL == PMC_NPD4) || (SET_PDMSEL == PMC_SPD0) || (SET_PDMSEL == PMC_SPD1) || (SET_PDMSEL == PMC_DPD))
    {
        /* Enable wake-up pin PC.0 falling edge wake-up */
        PMC_EnableWKPIN(PMC_WKPIN0_FALLING);
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        goto lexit;
    }

    /* Enter to Power-down mode */
    if (SET_PDMSEL == PMC_NPD0)            printf("Enter to NPD0 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_NPD1)       printf("Enter to NPD1 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_NPD2)       printf("Enter to NPD2 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_NPD3)       printf("Enter to NPD3 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_NPD4)       printf("Enter to NPD4 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_SPD0)       printf("Enter to SPD0 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_SPD1)       printf("Enter to SPD1 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_DPD)       printf("Enter to DPD Power-Down ......\n");

    printf("Press any key to start test\n");

    getchar();

    PowerDownFunction();

    /* Waiting for PC.0 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);
}
