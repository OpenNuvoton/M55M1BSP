/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC dynamic tamper function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32IsTamper = FALSE;

void SYS_Init(void);
void UART_Init(void);

/**
 * @brief       IRQ Handler for RTCTAMPER Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTCTAMPER_IRQHandler is default IRQ of RTC TAMPER.
 */
NVT_ITCM void RTCTAMPER_IRQHandler(void)
{
    uint32_t u32FlagStatus, u32TAMPCAL, u32TAMPTIME;
    uint32_t i;

    /* Tamper interrupt occurred */
    if (RTC_GET_TAMPER_INT_FLAG(RTC))
    {
        u32FlagStatus = RTC_GET_TAMPER_INT_STATUS(RTC);

        for (i = 0; i < 6; i++)
        {
            if (u32FlagStatus & (0x1UL << (i + RTC_INTSTS_TAMP0IF_Pos)))
                printf(" Tamper %u Detected!!\n", i);
        }

        u32TAMPCAL = RTC->TAMPCAL;
        u32TAMPTIME = RTC->TAMPTIME;
        printf(" Tamper detected date/time: 20%u%u/%u%u/%u%u ",
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENYEAR_Msk) >> RTC_TAMPCAL_TENYEAR_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_YEAR_Msk) >> RTC_TAMPCAL_YEAR_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENMON_Msk) >> RTC_TAMPCAL_TENMON_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_MON_Msk) >> RTC_TAMPCAL_MON_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENDAY_Msk) >> RTC_TAMPCAL_TENDAY_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_DAY_Msk) >> RTC_TAMPCAL_DAY_Pos));
        printf("%u%u:%u%u:%u%u.\n\n",
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENHR_Msk) >> RTC_TAMPTIME_TENHR_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_HR_Msk) >> RTC_TAMPTIME_HR_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENMIN_Msk) >> RTC_TAMPTIME_TENMIN_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_MIN_Msk) >> RTC_TAMPTIME_MIN_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENSEC_Msk) >> RTC_TAMPTIME_TENSEC_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_SEC_Msk) >> RTC_TAMPTIME_SEC_Pos));

        RTC_CLEAR_TAMPER_INT_FLAG(RTC, u32FlagStatus);
        /* make sure that interrupt flag has been cleared. */
        u32FlagStatus = RTC_GET_TAMPER_INT_STATUS(RTC);
        s_u32IsTamper = TRUE;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Set X32_OUT(PF.4) and X32_IN(PF.5)*/
    SET_X32_IN_PF5();
    SET_X32_OUT_PF4();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Enable External LXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LXTEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(RTC0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set multi-function pins for RTC Tamper */
    SET_TAMPER0_PF6();
    SET_TAMPER1_PF7();
    SET_TAMPER2_PF8();
    SET_TAMPER3_PF9();
    SET_TAMPER4_PF10();
    SET_TAMPER5_PF11();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime, sGetTime;
    S_RTC_TIME_DATA_T *spInitTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|    RTC Dynamic Tamper Sample Code   |\n");
    printf("+-------------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Open RTC and start counting */
    sInitTime.u32Year       = 2023;
    sInitTime.u32Month      = 6;
    sInitTime.u32Day        = 8;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_MONDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    /* check rtc reset status */
    spInitTime = (RTC->INIT & RTC_INIT_ACTIVE_Msk) ? NULL : &sInitTime;

    if (RTC_Open(spInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        return -1;
    }

    RTC_GetDateAndTime(&sGetTime);
    printf("# Initial data/time is: %u/%02u/%02u %02u:%02u:%02u.\n",
           sGetTime.u32Year, sGetTime.u32Month, sGetTime.u32Day, sGetTime.u32Hour, sGetTime.u32Minute, sGetTime.u32Second);
    printf("# Please connect (tamper0 & tamper1) and (tamper2 & tamper3) and (tamper4 & tamper5) first.\n");
    printf("                    (PF.6 to PF.7)          (PF.8 to PF.9)         (PF.10 to PF.11)\n");
    printf("# Press any key to start test:\n\n");
    getchar();

    printf("# Check tamper date/time when tamper event occurred:\n\n");

    RTC_CLEAR_TAMPER_INT_FLAG(RTC, RTC_INTSTS_TAMP0IF_Msk | RTC_INTSTS_TAMP1IF_Msk | RTC_INTSTS_TAMP2IF_Msk |
                              RTC_INTSTS_TAMP3IF_Msk | RTC_INTSTS_TAMP4IF_Msk | RTC_INTSTS_TAMP5IF_Msk);

    RTC_DynamicTamperEnable(RTC_PAIR0_SELECT | RTC_PAIR1_SELECT | RTC_PAIR2_SELECT, RTC_TAMPER_DEBOUNCE_ENABLE, 0, 0);
    RTC_DynamicTamperConfig(RTC_2POW10_CLK, 1, 0, RTC_REF_RANDOM_PATTERN);

    s_u32IsTamper = FALSE;

    /* Enable RTC Tamper Interrupt */
    RTC_EnableInt(RTC_INTEN_TAMP1IEN_Msk | RTC_INTEN_TAMP3IEN_Msk | RTC_INTEN_TAMP5IEN_Msk);
    NVIC_EnableIRQ(RTCTAMPER_IRQn);

    while (1)
    {
        while (s_u32IsTamper == FALSE) {}

        s_u32IsTamper = FALSE;
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

