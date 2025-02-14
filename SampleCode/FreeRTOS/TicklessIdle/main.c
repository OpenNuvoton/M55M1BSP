/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show task delay with tickless idle
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/* Standard includes. */
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo application include. */
#include "partest.h"
#include "NuMicro.h"

#define TIME_DELAY_SLEEP  500
//#define TIME_DELAY_SLEEP  30
#define tickless_task_PRIORITY  (configMAX_PRIORITIES - 2)

#define LED     PH4

static void Tickless_task(void *pvParameters);

/*-----------------------------------------------------------*/

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware(void);

/*
 * CMSIS clock configuration function.
 */
extern void SystemCoreClockUpdate(void);

/*-----------------------------------------------------------*/

int main(void)
{
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    /* Create tickless task */
    if (xTaskCreate(Tickless_task, "Tickless_task", configMINIMAL_STACK_SIZE + 100, NULL, tickless_task_PRIORITY, NULL) != pdPASS)
    {
        printf("Task creation failed!\n");

        while (1);
    }

    printf("\r\nTick count :\n");

    /* Task Scheduler */
    vTaskStartScheduler();

    for (;;) {}
}

/* Tickless Task */
static void Tickless_task(void *pvParameters)
{
    for (;;)
    {
        printf("%d\n", xTaskGetTickCount());
        /* Toggle LED */
        LED ^= 1;
        vTaskDelay(TIME_DELAY_SLEEP);
    }
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    vParTestInitialise();
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    printf("vApplicationMallocFailedHook\n");
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
    taskDISABLE_INTERRUPTS();

    for (;;);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    printf("vApplicationMallocFailedHook\n");
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();

    for (;;);
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