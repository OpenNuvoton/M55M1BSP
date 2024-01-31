/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC function and displays current time to the UART console.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define T_5SEC              (5)  // 5 Sec For Tick Time
#define T_60SEC             (60) // 60 Sec
#define T_TICK_1_SEC        (1)  // 1 Sec

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32RTCTInt = 0;

void SYS_Init(void);
void UART_Init(void);

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

    if (RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);

        s_u32RTCTInt = 1;

        PA2 ^= 1;
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
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(RTC0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    SET_GPIO_PA2();

}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime, sCurTime;
    uint32_t u32Sec;
    uint8_t u8IsNewDateTime = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);
    printf("+-----------------------------------+\n");
    printf("|    RTC Time Display Sample Code   |\n");
    printf("+-----------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Time Setting */
    sInitTime.u32Year       = 2023;
    sInitTime.u32Month      = 6;
    sInitTime.u32Day        = 8;
    sInitTime.u32Hour       = 13;
    sInitTime.u32Minute     = 0;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_MONDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    if (RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");

        while (1);
    }

    /* Enable RTC tick interrupt, one RTC tick is 1 second */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    printf("# Showing RTC date/time on DEBUG_PORT.\n\n");
    printf("1.) Use PA.2 to check tick period time is one second or not.\n");
    printf("2.) Show RTC date/time and change date/time after 5 seconds:\n");

    /* Use PA.2 to check tick period time */
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;

    u32Sec = 0;
    s_u32RTCTInt = 0;

    while (1)
    {
        if (s_u32RTCTInt == T_TICK_1_SEC)
        {
            s_u32RTCTInt = 0;

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sCurTime);
            printf("    %u/%02u/%02u %02u:%02u:%02u\n",
                   sCurTime.u32Year, sCurTime.u32Month, sCurTime.u32Day, sCurTime.u32Hour, sCurTime.u32Minute, sCurTime.u32Second);

            if (u32Sec == sCurTime.u32Second)
            {
                printf("\nRTC time is incorrect.\n");

                while (1);
            }

            u32Sec = sCurTime.u32Second;

            if (u8IsNewDateTime == 0)
            {
                if (u32Sec == sInitTime.u32Second + T_5SEC)
                {
                    printf("\n");
                    printf("3.) Update new date/time to 2023/6/9 13:12:11.\n");

                    u8IsNewDateTime = 1;
                    RTC_SetDate(2023, 6, 9, RTC_WEDNESDAY);
                    RTC_SetTime(13, 12, 11, RTC_CLOCK_24, RTC_AM);
                }
            }
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

