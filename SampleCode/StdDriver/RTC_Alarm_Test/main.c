/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC alarm function. It sets an alarm 10 seconds after execution.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define T_AlarmTime     10 //10Sec For Alarm Time
#define T_60SEC         60 //60Sec

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t s_u8IsAlarm = FALSE;

void RTC_AlarmHandle(void);
void RTC_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Handle                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_AlarmHandle(void)
{
    printf(" Alarm!!\n");
    s_u8IsAlarm = TRUE;
}

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC.
 */
NVT_ITCM void RTC_IRQHandler(void)
{
    uint32_t intflag;

    /* To check if RTC alarm interrupt occurred */
    if (RTC_GET_ALARM_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG(RTC);

        RTC_AlarmHandle();
    }

    if (RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);
    }

    /* make sure that interrupt flag has been cleared. */
    intflag = RTC->INTSTS;
    NVT_UNUSED(intflag);
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

}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime, sCurTime;
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
    printf("+----------------------------------+\n");
    printf("|    RTC Alarm Test Sample Code    |\n");
    printf("+----------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Time Setting */
    sInitTime.u32Year       = 2023;
    sInitTime.u32Month      = 6;
    sInitTime.u32Day        = 8;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 0;
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

    printf("\nRTC Alarm Test (Alarm after 10 seconds)\n\n");

    s_u8IsAlarm = FALSE;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf("Current Time: %u/%02u/%02u %02u:%02u:%02u\n",
           sCurTime.u32Year, sCurTime.u32Month, sCurTime.u32Day,
           sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    /* The alarm time setting */
    sCurTime.u32Second = sCurTime.u32Second + T_AlarmTime;

    if (sCurTime.u32Second >= T_60SEC)
    {
        sCurTime.u32Minute = sCurTime.u32Minute + (sCurTime.u32Second / T_60SEC);
        sCurTime.u32Second = sCurTime.u32Second % T_60SEC;
    }

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sCurTime);

    /* Clear RTC alarm interrupt flag */
    RTC_CLEAR_ALARM_INT_FLAG(RTC);

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    while (s_u8IsAlarm == FALSE) {}

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf("Current Time: %u/%02u/%02u %02u:%02u:%02u\n",
           sCurTime.u32Year, sCurTime.u32Month, sCurTime.u32Day,
           sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

    /* Disable RTC Alarm Interrupt */
    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    printf("\nRTC Alarm Test End !!\n");

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

