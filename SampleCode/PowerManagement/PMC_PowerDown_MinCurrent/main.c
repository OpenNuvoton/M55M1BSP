/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to minimize power consumption when entering Power-down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> NPD0
//      <1=> NPD1
//      <5=> SPD0
//      <7=> DPD0
*/
#define SET_PDMSEL    0

/*
// <o0> Voltage Regulator
//      <0=> LDO
//      <1=> DCDC
*/
#define SET_MVR       0

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
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
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

void PorSetting(void)
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

int32_t LircSetting(void)
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

int32_t LxtSetting(void)
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

int32_t VoltageRegulatorSetting(void)
{
    if (SET_MVR == 0)
    {
        /* Set voltage regulator to LDO mode */
        if (PMC_SetPowerRegulator(PMC_VRCTL_MVRS_LDO) != 0)
        {
            printf("Set voltage regulator to LDO mode not finished!\n");
            return -1;
        }
    }
    else
    {
        /* Set voltage regulator to DCDC mode */
        if (PMC_SetPowerRegulator(PMC_VRCTL_MVRS_DCDC) != 0)
        {
            printf("Set voltage regulator to DCDC mode not finished!\n");
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
    if (u32PMUSTS & (PMC_INTSTS_PIN0WKIF_Msk | PMC_INTSTS_GPCTGWKIF_Msk))
    {
        printf("System waken-up done.\n\n");

        while (1);
    }

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

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFP0 = 0;
    SYS->GPA_MFP1 = 0;
    SYS->GPA_MFP2 = 0;
    SYS->GPA_MFP3 = 0;
    SYS->GPB_MFP0 = 0;
    SYS->GPB_MFP1 = 0;
    SYS->GPB_MFP2 = 0;
    SYS->GPB_MFP3 = 0;
    SYS->GPC_MFP0 = 0;
    SYS->GPC_MFP1 = 0;
    SYS->GPC_MFP2 = 0;
    SYS->GPC_MFP3 = 0;
    SYS->GPD_MFP0 = 0;
    SYS->GPD_MFP1 = 0;
    SYS->GPD_MFP2 = 0;
    SYS->GPD_MFP3 = 0;
    SYS->GPE_MFP0 = 0;
    SYS->GPE_MFP1 = 0;
    SYS->GPE_MFP2 = 0;
    SYS->GPE_MFP3 = 0;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;
    SYS->GPF_MFP1 = 0;
    SYS->GPF_MFP2 = 0;
    SYS->GPG_MFP0 = 0;
    SYS->GPG_MFP1 = 0;
    SYS->GPG_MFP2 = 0;
    SYS->GPG_MFP3 = 0;
    SYS->GPH_MFP0 = 0;
    SYS->GPH_MFP1 = SYS_GPH_MFP1_PH5MFP_UART6_RXD | SYS_GPH_MFP1_PH4MFP_UART6_TXD;
    SYS->GPH_MFP2 = 0;
    SYS->GPH_MFP3 = 0;
    SYS->GPI_MFP1 = 0;
    SYS->GPI_MFP2 = 0;
    SYS->GPI_MFP3 = 0;
    SYS->GPJ_MFP0 = 0;
    SYS->GPJ_MFP1 = 0;
    SYS->GPJ_MFP2 = 0;
    SYS->GPJ_MFP3 = 0;
    /*
        Configure all GPIO as Quasi-bidirectional Mode. They are default output high.

        On NuMaker board, configure GPIO as input mode pull-down if they have pull-down resistor outside:
        TBD.
    */
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

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if (LircSetting() < 0) goto lexit;

    /* LXT setting */
    if (LxtSetting() < 0) goto lexit;

    /* Voltage regulator setting */
    if (VoltageRegulatorSetting() < 0) goto lexit;

    /* Wake-up source configuration */
    if (
        (SET_PDMSEL == PMC_NPD0)
        ||
        (SET_PDMSEL == PMC_NPD1)
#if 0   // TESTCHIP_ONLY not support    
        ||
        (SET_PDMSEL == PMC_NPD2)
#endif
    )
    {
        /* Configure PC.0 as Quasi mode and enable interrupt by falling edge trigger */
        CLK_EnableModuleClock(GPIOC_MODULE);
        GPIO_SetMode(PC, BIT0, GPIO_MODE_QUASI);
        GPIO_EnableInt(PC, 0, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPC_IRQn);
    }
    else if (
#if 0   // TESTCHIP_ONLY not support
        (SET_PDMSEL == PMC_NPD3)
        ||
        (SET_PDMSEL == PMC_NPD4)
        ||
#endif
        (SET_PDMSEL == PMC_SPD0)
#if 0   // TESTCHIP_ONLY not support        
        ||
        (SET_PDMSEL == PMC_SPD1)
#endif
    )
    {
        /* Enable wake-up pin PC.0 falling edge wake-up at SPD mode */
        PMC_EnableTGPin(PMC_TGPIN_PC, 0, PMC_TGPIN_FALLING, PMC_TGPIN_DEBOUNCEDIS, PMC_TGPIN_WAKEUP_ENABLE);
    }
    else if (
        (SET_PDMSEL == PMC_DPD0)
#if 0   // TESTCHIP_ONLY not support
        ||
        (SET_PDMSEL == PMC_DPD1)
#endif
    )
    {
        /* Enable wake-up pin PC.0 falling edge wake-up at DPD mode. PC.0 would be input mode floating at DPD mode. */
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

#if 0   // TESTCHIP_ONLY not support    
    else if (SET_PDMSEL == PMC_NPD2)       printf("Enter to NPD2 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_NPD3)       printf("Enter to NPD3 Power-Down ......\n");
    else if (SET_PDMSEL == PMC_NPD4)       printf("Enter to NPD4 Power-Down ......\n");

#endif
    else if (SET_PDMSEL == PMC_SPD0)       printf("Enter to SPD0 Power-Down ......\n");

#if 0   // TESTCHIP_ONLY not support    
    else if (SET_PDMSEL == PMC_SPD1)       printf("Enter to SPD1 Power-Down ......\n");

#endif

    else if (SET_PDMSEL == PMC_DPD0)       printf("Enter to DPD0 Power-Down ......\n");

#if 0   // TESTCHIP_ONLY not support
    else if (SET_PDMSEL == PMC_DPD1)       printf("Enter to DPD1 Power-Down ......\n");

#endif
    printf("Press any key to start test\n");

    getchar();

    PowerDownFunction();

    /* Waiting for PC.0 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);
}
