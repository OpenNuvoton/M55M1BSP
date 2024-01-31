/**************************************************************************//**
 * @file    main_ns.c
 * @version V1.00
 * @brief   Non-secure sample code for Collaborative Secure Software Development
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <arm_cmse.h>
#include "NuMicro.h"        /* Device header */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "nsc_functions.h"

/* Demo includes. */
#include "tz_demo.h"

/**
 * @brief Create all demo tasks.
 */
static void prvCreateTasks(void);

/*---------------------------------------------------------------------------
 * Main function
 *---------------------------------------------------------------------------*/
int main(void)
{
    printf("+-----------------------------------------+\n");
    printf("|       Non-secure code is running ffff       |\n");
    printf("+-----------------------------------------+\n");

    /* Init GPIO Port C Pin 0 for Non-secure LED control */
    GPIO_SetMode(PC_NS, BIT0, GPIO_MODE_OUTPUT);

    /* Create tasks. */
    prvCreateTasks();

    /* Start scheduler. */
    vTaskStartScheduler();

    /* Waiting for Secure/Non-secure SysTick interrupt */
    while (1);
}

/*-----------------------------------------------------------*/

static void prvCreateTasks(void)
{
    /* Create tasks for the TZ Demo. */
    vStartTZDemo();

}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()).  The code in this
    tick hook implementation is for demonstration only - it has no real
    purpose.  It just gives a semaphore every 50ms.  The semaphore unblocks a
    task that then toggles an LED.  Additionally, the call to
    vQueueSetAccessQueueSetFromISR() is part of the "standard demo tasks"
    functionality. */
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    printf("vApplicationMallocFailedHook \n");
    taskDISABLE_INTERRUPTS();

    for (;;);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
