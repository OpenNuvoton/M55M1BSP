/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up MCU from Power-down mode by ACMP wake-up function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define NPD0_MODE   0    // Power-down mode 0
#define NPD1_MODE   1    // Power-down mode 1
#define NPD3_MODE   2    // Power-down mode 3
#define SPD0_MODE   3    // Standby Power-down mode

/*---------------------------------------------------------------------------------------------------------*/
/*                                       Global variables                                                  */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t  g_i32WakeUp = FALSE;

/*---------------------------------------------------------------------------------------------------------*/
/*                                 Define functions prototype                                              */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void ACMP01_IRQHandler(void);
NVT_ITCM void PMC_IRQHandler(void);
void EnterToPowerDown(uint32_t u32PDMode);
void SYS_Init(void);
int32_t main(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

int IsDebugFifoEmpty(void)
{
    if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) != 0)
        return 0;
    else
        return 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         ACMP01  Handle                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void ACMP01_IRQHandler(void)
{
    printf("\nACMP1 interrupt!\n");
    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Clear wake-up interrupt flag */
    ACMP_CLR_WAKEUP_INT_FLAG(ACMP01, 1);

    UART_WAIT_TX_EMPTY(DEBUG_PORT);
}
/*---------------------------------------------------------------------------------------------------------*/
/*                                         PWRWU  Handle                                                   */
/*---------------------------------------------------------------------------------------------------------*/

NVT_ITCM void PMC_IRQHandler(void)
{
    if (PMC->INTSTS &  PMC_INTSTS_PDWKIF_Msk)
    {
        printf("PMC Wake up\n");
        PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;
        g_i32WakeUp = TRUE;
    }

    UART_WAIT_TX_EMPTY(DEBUG_PORT);

}
/**
  * @brief      Enter To Power Down
  * @param[in]   u32PDMode    The specified Power down module.
  *                               - \ref CLK_PMUCTL_PDMSEL_PD      : Power-down mode
  *                               - \ref CLK_PMUCTL_PDMSEL_FWPD    : Fast wake up
  *
  * @return     None
  *
  * @details    This API is used to get the current RTC date and time value.
  */

void EnterToPowerDown(uint32_t u32PDMode)
{
    g_i32WakeUp = FALSE;
    SYS_UnlockReg();
    /* To program PWRCTL register, it needs to disable register protection first. */
    PMC->INTEN &= ~PMC_INTEN_PDWKIEN_Msk;

    if (u32PDMode == NPD0_MODE)
        PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL0);   //Power down
    else if (u32PDMode == NPD1_MODE)
        PMC_SetPowerDownMode(PMC_NPD1, PMC_PLCTL_PLSEL_PL0);   //Power down
    else if (u32PDMode == NPD3_MODE)
        PMC_SetPowerDownMode(PMC_NPD3, PMC_PLCTL_PLSEL_PL0);   //Power down
    else if (u32PDMode == SPD0_MODE)
        PMC_SetPowerDownMode(PMC_SPD0, PMC_PLCTL_PLSEL_PL0);   //Power down

    PMC->INTEN |= PMC_INTEN_PDWKIEN_Msk;

    PMC_PowerDown();
    SYS_LockReg();

    while (g_i32WakeUp == FALSE);

}



void SYS_Init(void)
{

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Set PCLK1 divide 4 */
    CLK_SET_PCLK1DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);
    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Enable GPC peripheral clock */
    CLK_EnableModuleClock(GPIOC_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB.4 and PC.0 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE4_Msk);
    PC->MODE &= ~(GPIO_MODE_MODE0_Msk);

    /* Set PB4 multi-function pin for ACMP1 positive input pin and PC0 multi-function pin for ACMP1 output pin*/
    SET_ACMP1_P1_PB4();
    SET_ACMP1_O_PC0();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));

}

/*
 * When the voltage of the positive input is greater than the voltage of the negative input,
 * the analog comparator outputs logical one; otherwise, it outputs logical zero.
 * This chip will be waked up from power down mode when detecting a transition of analog comparator's output.
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

    /* Select SCLK to HIRC before APLL setting*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    /* Lock protected registers */
    SYS_LockReg();

    printf("\nThis sample code demonstrates ACMP1 function. Using ACMP1_P1 (PB4) as ACMP1\n");
    printf("positive input and using internal CRV as the negative input.\n");
    printf("The compare result reflects on ACMP1_O (PC0).\n");
    printf("Press any key to enter power down mode ...\n");
    getchar();

    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV1_SRC(ACMP01, ACMP_VREF_CRV1SSEL_VDDA);
    /* Select CRV1 level: VDDA * 31 / 63 */
    ACMP_CRV1_SEL(ACMP01, 31);

    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);

    CLK_SysTickDelay(10); /* For ACMP setup time */

    /* Clear ACMP 0 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Enable wake-up function */
    ACMP_ENABLE_WAKEUP(ACMP01, 1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);
    /* Enable PMC interrupt */
    NVIC_EnableIRQ(PMC_IRQn);
    printf("\nSystem enter power-down mode ... \n");

    /* To check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    EnterToPowerDown(NPD0_MODE);

    printf("Wake up by ACMP1!\n");

    ACMP_DISABLE_WAKEUP(ACMP01, 1);

    NVIC_DisableIRQ(ACMP01_IRQn);

    NVIC_DisableIRQ(PMC_IRQn);

    while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
