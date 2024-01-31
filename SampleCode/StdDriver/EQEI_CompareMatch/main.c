/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Show the usage of EQEI compare function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define EQEI0A   PC3
#define EQEI0B   PC4

/**
 * @brief       IRQ Handler for EQEI0 Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The EQEI0_IRQHandler is default IRQ of EQEI0, declared in startup_M55M1.c.
 */
NVT_ITCM void EQEI0_IRQHandler(void)
{
    uint32_t intflag;

    if (EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_CMPF_Msk))    /* Compare-match flag */
    {
        printf("Compare-match INT!\n\n");
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_CMPF_Msk);
    }

    if (EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_OVUNF_Msk))   /* Counter Overflow or underflow flag */
    {
        printf("Overflow INT!\n\n");
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_OVUNF_Msk);
    }

    /* make sure that interrupt flag has been cleared. */
    intflag = EQEI0->STATUS;
    NVT_UNUSED(intflag);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(EQEI0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pins for GPIO*/
    SET_GPIO_PC3();
    SET_GPIO_PC4();
    /* Set PA multi-function pins for EQEI0_A, EQEI0_B*/
    SET_EQEI0_A_PD11();
    SET_EQEI0_B_PA3();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);

    printf("+--------------------------------------+\n");
    printf("|     EQEI Driver Sample Code          |\n");
    printf("+--------------------------------------+\n\n");
    printf("  >> Please connect PC.3 and PD.11 << \n");
    printf("  >> Please connect PC.4 and PA.3  << \n");
    printf("     Press any key to start test\n\n");
    getchar();
    printf("     start test......\n\n");
    /* Configure PC.3 and PC.4 as output mode */
    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);

    EQEI0A = 0;
    EQEI0B = 0;

    /* Set EQEI counting mode as X4 Compare-counting mode,
       set maximum counter value and enable IDX, QEA and QEB input */
    EQEI_Open(EQEI0, EQEI_CTL_X4_COMPARE_COUNTING_MODE, 0x20000);

    /* Set counter compare value */

    EQEI_SET_CNT_CMP(EQEI0, 0x10000);

    /* Enable compare function */
    EQEI_ENABLE_CNT_CMP(EQEI0);

    /* Enable EQEI interrupt */
    EQEI_EnableInt(EQEI0, EQEI_CTL_CMPIEN_Msk | EQEI_CTL_OVUNIEN_Msk);

    /* Enable EQEI IRQ */
    NVIC_EnableIRQ(EQEI0_IRQn);

    /* Start EQEI function */
    EQEI_Start(EQEI0);

    /* Wait compare-match and overflow interrupt happened */
    while (1)
    {
        EQEI0A = 1;
        CLK_SysTickDelay(16);
        EQEI0B = 1;
        CLK_SysTickDelay(16);
        EQEI0A = 0;
        CLK_SysTickDelay(16);
        EQEI0B = 0;
        CLK_SysTickDelay(16);
    }

}
