/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of ACMP window compare function
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
    /* Clear interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    if (ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage is within the window\n");
    }
    else
    {
        printf("The input voltage is not within the window\n");
    }
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
    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPIOC_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PA.10 to input mode */
    GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);
    /* Set PA.11 to input mode */
    GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);

    /* Set PA11 multi-function pin for ACMP0 positive input pin */
    SET_ACMP0_P0_PA11();

    /* Set PA10 multi-function pin for ACMP1 positive input pin */
    SET_ACMP1_P0_PA10();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Disable digital input path of analog pin ACMP0_P0 and ACMP1_P0 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 11));
    GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 10));
}


int32_t main(void)
{
    uint32_t volatile u32DelayCount;

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

    printf("\nThis sample code demonstrates ACMP window compare function\n");
    printf("Connect the specific analog voltage source to the positive inputs\n");
    printf("of both comparators, PA10 and PA11. This sample code will monitor if the\n");
    printf("input is between the range of VDDA * 40 / 63 and bandgap.\n");
    printf("Press any key to continue ...\n");
    getchar();

    /* Configure ACMP0. Enable ACMP0 and select VBG as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 0, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV1_SRC(ACMP01, ACMP_VREF_CRV1SSEL_VDDA);
    /* Select CRV1 level: VDDA * 40 / 63 */
    ACMP_CRV1_SEL(ACMP01, 40);
    /* Select P0 as ACMP0 positive input channel */
    ACMP_SELECT_P(ACMP01, 0, ACMP_CTL_POSSEL_P0);
    /* Select P0 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P0);
    /* Enable window compare mode */
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 0);
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 1);

    /* Clear ACMP 0 and 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    // Give ACMP some time to settle
    for (u32DelayCount = 0; u32DelayCount < 1000; u32DelayCount++);

    if (ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage in inside the window\n");
    }
    else
    {
        printf("The input voltage in outside the window\n");
    }

    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


