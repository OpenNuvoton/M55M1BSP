/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/* Scheduler includes. */

#include <string.h>
#include <time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "NuMicro.h"

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ             ( configCPU_CLOCK_HZ )
    /* Ensure the SysTick is clocked at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT_CONFIG    ( portNVIC_SYSTICK_CLK_BIT )
#else
    /* Select the option to clock SysTick not at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT_CONFIG    ( 0 )
#endif

/**
 * @brief Constants required to manipulate the NVIC.
 */
#define portNVIC_SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SHPR3_REG                    ( *( ( volatile uint32_t * ) 0xe000ed20 ) )
#define portNVIC_SYSTICK_ENABLE_BIT           ( 1UL << 0UL )
#define portNVIC_SYSTICK_INT_BIT              ( 1UL << 1UL )
#define portNVIC_SYSTICK_CLK_BIT              ( 1UL << 2UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT       ( 1UL << 16UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT       ( 1UL << 25UL )
#define portNVIC_PEND_SYSTICK_SET_BIT         ( 1UL << 26UL )
#define portMIN_INTERRUPT_PRIORITY            ( 255UL )
#define portNVIC_PENDSV_PRI                   ( portMIN_INTERRUPT_PRIORITY << 16UL )
#define portNVIC_SYSTICK_PRI                  ( portMIN_INTERRUPT_PRIORITY << 24UL )

/*-----------------------------------------------------------*/

/**
 * @brief A fiddle factor to estimate the number of SysTick counts that would
 * have occurred while the SysTick counter is stopped during tickless idle
 * calculations.
 */
#define portMISSED_COUNTS_FACTOR    ( 94UL )
/*-----------------------------------------------------------*/

/**
 * @brief The minimum requirement of RTC slepp time.
 */
#define MIN_RTC_SLEEP_TIME (25UL)

#if ( configUSE_TICKLESS_IDLE == 1 )

/*
 * The number of SysTick increments that make up one tick period.
 */
static uint32_t ulTimerCountsForOneTick = 0;

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
static uint32_t ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);

extern void PowerDownFunction(void);

#define RTC_TIME_HZCNT_Pos               (24)                                              /*!< RTC_T::TIME: HZCNT Position            */
#define RTC_TIME_HZCNT_Msk               (0x7ful << RTC_TIME_HZCNT_Pos)                    /*!< RTC_T::TIME: HZCNT Mask                */

#define RTC_TALM_HZCNT_Pos               (24)                                              /*!< RTC_T::TALM: HZCNT Position            */
#define RTC_TALM_HZCNT_Msk               (0x7ful << RTC_TALM_HZCNT_Pos)                    /*!< RTC_T::TALM: HZCNT Mask                */

static void TimeAdd(S_RTC_TIME_DATA_T *t, volatile uint32_t *pu32RtcTicks, uint32_t u32rtcTicks)
{
    uint32_t h, m, s, tick;
    int32_t i32Next;

    m = u32rtcTicks / 128 / 60;
    h = m / 60;
    m = m % 60;
    s = u32rtcTicks / 128 % 60;
    tick = u32rtcTicks & 0x7f;

    *pu32RtcTicks += tick;
    t->u32Second += s;
    t->u32Minute += m;
    t->u32Hour += h;

    do
    {
        i32Next = 0;

        if (*pu32RtcTicks >= 128)
        {
            *pu32RtcTicks -= 128;
            t->u32Second += 1;
            i32Next = 1;
        }

        if (t->u32Second >= 60)
        {
            t->u32Second -= 60;
            t->u32Minute += 1;
            i32Next = 1;
        }

        if (t->u32Minute >= 60)
        {
            t->u32Minute -= 60;
            t->u32Hour += 1;
            i32Next = 1;
        }

        if (t->u32Hour >= 24)
        {
            t->u32Hour -= 24;
            t->u32Minute += 1;
            i32Next = 1;
        }
    } while (i32Next);
}

static TickType_t SleepProcessBySysTick(TickType_t x)
{
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements, ulSysTickDecrementsLeft;

    /* Use the SysTick current-value register to determine the number of
     * SysTick decrements remaining until the next tick interrupt.  If the
     * current-value register is zero, then there are actually
     * ulTimerCountsForOneTick decrements remaining, not zero, because the
     * SysTick requests the interrupt when decrementing from 1 to 0. */
    ulSysTickDecrementsLeft = portNVIC_SYSTICK_CURRENT_VALUE_REG;

    if (ulSysTickDecrementsLeft == 0)
    {
        ulSysTickDecrementsLeft = ulTimerCountsForOneTick;
    }

    /* Calculate the reload value required to wait xExpectedIdleTime
     * tick periods.  -1 is used because this code normally executes part
     * way through the first tick period.  But if the SysTick IRQ is now
     * pending, then clear the IRQ, suppressing the first tick, and correct
     * the reload value to reflect that the second tick period is already
     * underway.  The expected idle time is always at least two ticks. */
    ulReloadValue = ulSysTickDecrementsLeft + (ulTimerCountsForOneTick * (x - 1UL));

    if ((portNVIC_INT_CTRL_REG & portNVIC_PEND_SYSTICK_SET_BIT) != 0)
    {
        portNVIC_INT_CTRL_REG = portNVIC_PEND_SYSTICK_CLEAR_BIT;
        ulReloadValue -= ulTimerCountsForOneTick;
    }

    if (ulReloadValue > ulStoppedTimerCompensation)
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Set the new reload value. */
    portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

    /* Clear the SysTick count flag and set the count value back to
     * zero. */
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Restart SysTick. */
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

    /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
     * set its parameter to 0 to indicate that its implementation contains
     * its own wait for interrupt or wait for event instruction, and so wfi
     * should not be executed again.  However, the original expected idle
     * time variable must remain unmodified, so a copy is taken. */
    PMC_Idle();

    /* Re-enable interrupts to allow the interrupt that brought the MCU
     * out of sleep mode to execute immediately.  See comments above
     * the cpsid instruction above. */
    __enable_irq();
    __ISB();
    __DSB();

    /* Disable interrupts again because the clock is about to be stopped
     * and interrupts that execute while the clock is stopped will increase
     * any slippage between the time maintained by the RTOS and calendar
     * time. */
    __disable_irq();
    __DSB();
    __ISB();

    /* Disable the SysTick clock without reading the
     * portNVIC_SYSTICK_CTRL_REG register to ensure the
     * portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set.  Again,
     * the time the SysTick is stopped for is accounted for as best it can
     * be, but using the tickless mode will inevitably result in some tiny
     * drift of the time maintained by the kernel with respect to calendar
     * time*/
    portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT);

    /* Determine whether the SysTick has already counted to zero. */
    if ((portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0)
    {
        uint32_t ulCalculatedLoadValue;

        /* The tick interrupt ended the sleep (or is now pending), and
         * a new tick period has started.  Reset portNVIC_SYSTICK_LOAD_REG
         * with whatever remains of the new tick period. */
        ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL) - (ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG);

        /* Don't allow a tiny value, or values that have somehow
         * underflowed because the post sleep hook did something
         * that took too long or because the SysTick current-value register
         * is zero. */
        if ((ulCalculatedLoadValue <= ulStoppedTimerCompensation) || (ulCalculatedLoadValue > ulTimerCountsForOneTick))
        {
            ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL);
        }

        portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

        /* As the pending tick will be processed as soon as this
         * function exits, the tick value maintained by the tick is stepped
         * forward by one less than the time spent waiting. */
        ulCompleteTickPeriods = x - 1UL;
    }
    else
    {
        /* Something other than the tick interrupt ended the sleep. */

        /* Use the SysTick current-value register to determine the
         * number of SysTick decrements remaining until the expected idle
         * time would have ended. */
        ulSysTickDecrementsLeft = portNVIC_SYSTICK_CURRENT_VALUE_REG;
#if ( portNVIC_SYSTICK_CLK_BIT_CONFIG != portNVIC_SYSTICK_CLK_BIT )
        {
            /* If the SysTick is not using the core clock, the current-
             * value register might still be zero here.  In that case, the
             * SysTick didn't load from the reload register, and there are
             * ulReloadValue decrements remaining in the expected idle
             * time, not zero. */
            if (ulSysTickDecrementsLeft == 0)
            {
                ulSysTickDecrementsLeft = ulReloadValue;
            }
        }
#endif /* portNVIC_SYSTICK_CLK_BIT_CONFIG */

        /* Work out how long the sleep lasted rounded to complete tick
         * periods (not the ulReload value which accounted for part
         * ticks). */
        ulCompletedSysTickDecrements = (x * ulTimerCountsForOneTick) - ulSysTickDecrementsLeft;

        /* How many complete tick periods passed while the processor
         * was waiting? */
        ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

        /* The reload value is set to whatever fraction of a single tick
         * period remains. */
        portNVIC_SYSTICK_LOAD_REG = ((ulCompleteTickPeriods + 1UL) * ulTimerCountsForOneTick) - ulCompletedSysTickDecrements;
    }

    return ulCompleteTickPeriods;
}

static TickType_t SleepProcessByRTC(TickType_t x)
{
    S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
    uint32_t u32WriteRTC_Ticks, u32ReadRTC_Ticks;
    struct tm sRTCStartTM;
    struct tm sRTCEndTM;

    time_t tRTCStartSec;
    time_t tRTCEndSec;

    uint32_t u32CompletedSec;
    uint32_t u32CompletedRTCTick;
    uint32_t u32CompletedRTOSTick;

    if (x == 0)
        return x;

    /* Set RTC alarm accord to x */
    /* Open RTC and start counting */
    /* Get start time from RTC */
    RTC_GetDateAndTime(&sReadRTC);
    u32ReadRTC_Ticks = ((RTC->TIME & RTC_TIME_HZCNT_Msk) >> RTC_TIME_HZCNT_Pos);
    sRTCStartTM.tm_year = sReadRTC.u32Year - 1900;
    sRTCStartTM.tm_mon = sReadRTC.u32Month - 1;
    sRTCStartTM.tm_mday = sReadRTC.u32Day;
    sRTCStartTM.tm_hour = sReadRTC.u32Hour;
    sRTCStartTM.tm_min = sReadRTC.u32Minute;
    sRTCStartTM.tm_sec = sReadRTC.u32Second;

    tRTCStartSec = mktime(&sRTCStartTM);

    /* Backup RTC start time for GPIO wake up */
    memcpy(&sWriteRTC, &sReadRTC, sizeof(S_RTC_TIME_DATA_T));
    u32WriteRTC_Ticks = u32ReadRTC_Ticks;

    /* Calculate idle end time for RTC alarm wake up */
    TimeAdd(&sReadRTC, &u32WriteRTC_Ticks, ((x * portTICK_PERIOD_MS) * 128 / 1000) - 1);

    /* Set wake up alarm */
    RTC_SetAlarmDateAndTime(&sReadRTC);
    RTC->TALM |= (RTC->TALM & (~RTC_TALM_HZCNT_Msk)) | (u32WriteRTC_Ticks << RTC_TALM_HZCNT_Pos);

    /* Enable RTC alarm interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    PowerDownFunction();

    /* Wake up now. Enable interrupt for process the pending interrupt handler */
    __enable_irq();
    __ISB();
    __DSB();

    /* Disable interrupts again because the clock is about to be stopped
     * and interrupts that execute while the clock is stopped will increase
     * any slippage between the time maintained by the RTOS and calendar
     * time. */
    __disable_irq();
    __DSB();
    __ISB();

    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);


    RTC_GetDateAndTime(&sReadRTC);
    u32CompletedRTCTick = ((RTC->TIME & RTC_TIME_HZCNT_Msk) >> RTC_TIME_HZCNT_Pos);

    sRTCEndTM.tm_year = sReadRTC.u32Year - 1900;
    sRTCEndTM.tm_mon = sReadRTC.u32Month - 1;
    sRTCEndTM.tm_mday = sReadRTC.u32Day;
    sRTCEndTM.tm_hour = sReadRTC.u32Hour;
    sRTCEndTM.tm_min = sReadRTC.u32Minute;
    sRTCEndTM.tm_sec = sReadRTC.u32Second;

    tRTCEndSec = mktime(&sRTCEndTM);
    u32CompletedSec = tRTCEndSec - tRTCStartSec;


    if (u32CompletedRTCTick < u32ReadRTC_Ticks)
    {
        u32CompletedRTCTick = u32CompletedRTCTick + 128 - u32ReadRTC_Ticks;
        u32CompletedSec --;
    }
    else
    {
        u32CompletedRTCTick = u32CompletedRTCTick - u32ReadRTC_Ticks;
    }

    u32CompletedRTOSTick = (u32CompletedRTCTick * 1000 / 128) / portTICK_PERIOD_MS;
    u32CompletedRTOSTick += (u32CompletedSec * 1000 / portTICK_PERIOD_MS);

    return u32CompletedRTOSTick;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t ulCompleteTickPeriods;
    ulTimerCountsForOneTick = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);

    /* Stop the SysTick momentarily. The time the SysTick is stopped for
     * is accounted for as best it can be, but using the tickless mode will
     * inevitably result in some tiny drift of the time maintained by the
     * kernel with respect to calendar time. */
    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
     * method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();
    __DSB();
    __ISB();

    /* If a context switch is pending or a task is waiting for the scheduler
     * to be unsuspended then abandon the low power entry. */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep)
    {

        /* Re-enable interrupts - see comments above the cpsid instruction()
         * above. */
        __enable_irq();
    }
    else
    {
        TickType_t xIdleTicks;
        uint32_t u32ExpectedIdleMS;

        /* Stop the SysTick momentarily.  The time the SysTick is stopped for
         * is accounted for as best it can be, but using the tickless mode will
         * inevitably result in some tiny drift of the time maintained by the
         * kernel with respect to calendar time. */
        portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT);

        /* Sleep until something happens. */
        u32ExpectedIdleMS = xExpectedIdleTime * portTICK_PERIOD_MS;

        if (u32ExpectedIdleMS > MIN_RTC_SLEEP_TIME)
        {
            xIdleTicks = SleepProcessByRTC(xExpectedIdleTime);
        }
        else
        {
            xIdleTicks = SleepProcessBySysTick(xExpectedIdleTime);
        }

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG again,
         * then set portNVIC_SYSTICK_LOAD_REG back to its standard value.  If
         * the SysTick is not using the core clock, temporarily configure it to
         * use the core clock.  This configuration forces the SysTick to load
         * from portNVIC_SYSTICK_LOAD_REG immediately instead of at the next
         * cycle of the other clock.  Then portNVIC_SYSTICK_LOAD_REG is ready
         * to receive the standard value immediately. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
#if ( portNVIC_SYSTICK_CLK_BIT_CONFIG == portNVIC_SYSTICK_CLK_BIT )
        {
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
        }
#else
        {
            /* The temporary usage of the core clock has served its purpose,
             * as described above.  Resume usage of the other clock. */
            portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT;

            if ((portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0)
            {
                /* The partial tick period already ended.  Be sure the SysTick
                 * counts it only once. */
                portNVIC_SYSTICK_CURRENT_VALUE_REG = 0;
            }

            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
            portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT_CONFIG | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
        }
#endif /* portNVIC_SYSTICK_CLK_BIT_CONFIG */

        if (xIdleTicks < xExpectedIdleTime)
        {
            ulCompleteTickPeriods = xIdleTicks;
        }
        else
        {
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }

        //printf("ddddd vPortSuppressTicksAndSleep xIdleTicks: %d \n", xIdleTicks);
        vTaskStepTick(ulCompleteTickPeriods);

        /* Exit with interrpts enabled. */
        __enable_irq();
    }
}

#endif /* configUSE_TICKLESS_IDLE */
