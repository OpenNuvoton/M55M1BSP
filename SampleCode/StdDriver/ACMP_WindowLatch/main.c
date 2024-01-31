/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of ACMP window latch function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void ACMP01_IRQHandler(void);
void SYS_Init(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

NVT_ITCM void ACMP01_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Check Comparator 1 Output Status */
    if (ACMP_GET_OUTPUT(ACMP01, 1))
        printf("ACMP1_P voltage > Band-gap voltage (%u) ACMP1_O(%d)\n", u32Cnt, PC0);
    else
        printf("ACMP1_P voltage <= Band-gap voltage (%u) ACMP1_O(%d)\n", u32Cnt, PC0);

    u32Cnt++;
}


void SYS_Init(void)
{

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Set PCLK1 divide 4 */
    CLK_SET_PCLK1DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);
    /* Enable GPA peripheral clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPIOC_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PA.10 and PC.0 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE10_Msk);
    PC->MODE &= ~(GPIO_MODE_MODE0_Msk);

    /* Set PA6 multi-function pin for ACMP1 window latch pin */
    SET_ACMP1_WLAT_PA6();
    /* Set PA10 multi-function pin for ACMP1 positive input pin and PC0 multi-function pin for ACMP1 output pin*/
    SET_ACMP1_P0_PA10();
    SET_ACMP1_O_PC0();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Disable digital input path of analog pin ACMP1_P0 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 10));
}

/*
 * In window latch mode and window latch pin is at high level, the voltage of the positive
 * input is greater than the  voltage of the negative input, the analog comparator outputs
 * logical one; otherwise, it outputs logical zero. When window latch pin is at low level,
 * the output level does not change no matter how positive  input voltage changes.
 */

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nThis sample code demonstrates ACMP1 window latch function. Using ACMP1_P0 (PA10) as ACMP1\n");
    printf("positive input and using internal band-gap voltage as the negative input. ACMP1_WLAT is at\n");
    printf("PA6, when PA6 is low, compare result on ACMP1_O (PC0) does not change with ACMP1_P0. When PA6");
    printf("is high, ACMP1_O works as usual\n");

    printf("Press any key to start ...\n");
    getchar();
    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    //    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_PIN, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P0 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P0);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable window latch mode */
    ACMP_ENABLE_WINDOW_LATCH(ACMP01, 1);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
