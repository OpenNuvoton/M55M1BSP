/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * https://aws.amazon.com/freertos
 *
 */
#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Non-Secure callable functions. */
#include "nsc_functions.h"

#define NUM_TASKS   4

/**
 * @brief Counter incremented in the callback which is called from the secure
 * side.
 *
 * The size of an MPU region must be a multiple of 32 bytes. Therefore we need
 * to declare an array of size 8 to ensure that the total size is 32 bytes -
 * even though we only need 4 bytes. If we do not do that, anything placed after
 * 4 bytes and upto 32 bytes will also fall in the same MPU region and the task
 * having access to ulNonSecureCounter will also have access to all those items.
 */
static uint32_t ulNonSecureCounter[ NUM_TASKS ] __attribute__((aligned(32))) = { 0 };
/*-----------------------------------------------------------*/

/**
 * @brief Creates all the tasks for TZ demo.
 */
void vStartTZDemo(void);

/**
 * @brief Increments the ulNonSecureCounter.
 *
 * This function is called from the secure side.
 */
static void prvCallback(void *pvParameters);

/**
 * @brief Implements the task which calls the functions exported from the secure
 * side.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvSecureCallingTask(void *pvParameters);
/*-----------------------------------------------------------*/

void vStartTZDemo(void)
{
    int i;

    for (i = 0 ; i < NUM_TASKS; i ++)
        xTaskCreate(prvSecureCallingTask,
                    "SecCalling",
                    configMINIMAL_STACK_SIZE,
                    (void *) i,
                    tskIDLE_PRIORITY,
                    NULL);  /* FREERTOS_SYSTEM_CALL */

}
/*-----------------------------------------------------------*/

static void prvCallback(void *pvParameters)
{
    int32_t i32TaskNo = (int32_t)pvParameters;

    /* This function is called from the secure side. Just increment the counter
     * here. The check that this counter keeps incrementing is performed in the
     * prvSecureCallingTask. */
    ulNonSecureCounter[ i32TaskNo ] += 1;
}
/*-----------------------------------------------------------*/

static void prvSecureCallingTask(void *pvParameters)
{
    char szInfo[100];
    uint32_t ulLastSecureCounter = 0, ulLastNonSecureCounter = 0;
    uint32_t ulCurrentSecureCounter = 0;
    int32_t i32TaskNo = (int32_t)pvParameters;
    S_NSCFUN_PARAMETER sFunParam;

    sFunParam.i32TaskNo = i32TaskNo;
    sFunParam.ulSecureCounter = 0;

    /* This task calls secure side functions. So allocate a secure context for
     * it. */
    portALLOCATE_SECURE_CONTEXT(configMINIMAL_SECURE_STACK_SIZE);

    for (; ;)
    {
        /* Call the secure side function. It does two things:
         * - It calls the supplied function (prvCallback) which in turn
         *   increments the non-secure counter.
         * - It increments the secure counter and returns the incremented value.
         * Therefore at the end of this function call both the secure and
         * non-secure counters must have been incremented.
         */
        ulCurrentSecureCounter = NSCFunction(prvCallback, &sFunParam);

        /* Make sure that both the counters are incremented. */
        configASSERT(ulCurrentSecureCounter == ulLastSecureCounter + 1);
        configASSERT(ulNonSecureCounter[ i32TaskNo ] == ulLastNonSecureCounter + 1);

        sprintf(szInfo, "ulCurrentSecureCounter %d \n", (int32_t)ulCurrentSecureCounter);
        NSCPrintf(szInfo);
        sprintf(szInfo, "ulNonSecureCounter[ %d ] %d \n", i32TaskNo, ulNonSecureCounter[ i32TaskNo ]);
        NSCPrintf(szInfo);

        /* Update the last values for both the counters. */
        ulLastSecureCounter = ulCurrentSecureCounter;
        ulLastNonSecureCounter = ulNonSecureCounter[ i32TaskNo ];

        /* Wait for a second. */
        //        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/*-----------------------------------------------------------*/
