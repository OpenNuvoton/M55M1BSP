/**************************************************************************//**
 * @file     pmu_counter.h
 * @version  V1.00
 * @brief    Performacne monitor counters(PMU, SysTick) functions
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __PMU_COUNTER_H__
#define __PMU_COUNTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define NUM_PMU_COUNTERS     (10)     /**< Maximum number of available counters. */

/**
 * @brief   Container for a single unit for a PMU counter.
 */
typedef struct _pmu_counter_unit
{
    uint64_t value;     /**< Value of the counter expressed as 64 bits unsigned integer. */
    const char *name;   /**< Name for the counter. */
    const char *unit;   /**< Unit that the counter value represents (like cycles, beats, or milliseconds). */
} pmu_counter_unit;

/**
 * @brief   Container for a an array of counters
 */
typedef struct _pmu_counters
{
    pmu_counter_unit counters[NUM_PMU_COUNTERS]; /**< Counter array. */
    uint32_t num_counters;                       /**< Number of valid counters. */
    bool initialised;                            /**< Initialised or not. */
} pmu_counters;

/**
 * @brief   Resets the counters.
 */
void pmu_reset_counters(void);

/**
 * @brief       Gets the current counter values.
 * @param[out]  Pointer to a pmu_counters object.
 **/
void pmu_get_counters(pmu_counters *counters);

/**
 * @brief       Gets the systick counter value.
 * @param[out]  uint64_t systick counter value
 **/
uint64_t pmu_get_systick_Count(void);

/* For Arm profiler function */
#define hal_pmu_reset() pmu_reset_counters()
#define hal_pmu_get_counters(x) pmu_get_counters(x)

#ifdef __cplusplus
}
#endif
#endif
