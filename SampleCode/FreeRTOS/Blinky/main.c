
/******************************************************************************
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting (defined in this file) is used to
 * select between the two.  The simply blinky demo is implemented and described
 * in main_blinky.c.  The more comprehensive test and demo application is
 * implemented and described in main_full.c.
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and FreeRTOS hook functions.  It also contains a dummy
 * interrupt service routine called Dummy_IRQHandler() that is provided as an
 * example of how to use interrupt safe FreeRTOS API functions (those that end
 * in "FromISR").
 *
 *****************************************************************************/
#include <stdio.h>


/* Standard includes. */
#include "string.h"

/* Hardware includes. */
#include "NuMicro.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Demo application include. */
#include "partest.h"
#include "QueueSet.h"

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,
or 0 to run the more comprehensive test and demo application. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY  0


/*-----------------------------------------------------------*/

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware(void);

/*
 * main_blinky() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 * main_full() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 0.
 */
extern void main_blinky(void);
extern void main_full(void);

/*
 * CMSIS clock configuration function.
 */
extern void SystemCoreClockUpdate(void);
void vApplicationMallocFailedHook(void)__attribute__((noreturn));
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)__attribute__((noreturn));
void vApplicationTickHook(void);
/*-----------------------------------------------------------*/

int main(void)
{
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    printf("\nThis sample code test FreeRTOS function.\n");

    /* The mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting is described at the top
    of this file. */
#if mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 1
    {
        main_blinky();
    }
#else
    {
        main_full();
    }
#endif

    return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{

    vParTestInitialise();

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
    printf("vApplicationMallocFailedHook\n");
    taskDISABLE_INTERRUPTS();

    for (;;);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    printf("vApplicationStackOverflowHook\n");
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();

    //__BKPT();

    //printf("Stack overflow task name=%s\n", pcTaskName);


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

    /* The semaphore and associated task are not created when the simple blinky
    demo is used. */
#if mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0
    {
        static unsigned long ulLastGiveTime = 0UL;
        const unsigned long ulRate = 50UL / portTICK_PERIOD_MS;
        extern SemaphoreHandle_t xLEDSemaphore;

        configASSERT(xLEDSemaphore);

        if ((xTaskGetTickCountFromISR() - ulLastGiveTime) > ulRate)
        {
            /* The second parameter is normally used to determine if a context
            switch should be performed or not.  In this case the function is
            being performed from the tick hook, so the scheduler will make that
            assessment before returning to a task anyway - so the parameter is
            not needed and is just set to NULL. */
            xSemaphoreGiveFromISR(xLEDSemaphore, NULL);
            ulLastGiveTime += ulRate;
        }

        /* Write to a queue that is in use as part of the queue set demo to
        demonstrate using queue sets from an ISR. */
        vQueueSetAccessQueueSetFromISR();
    }
#endif /* mainCREATE_SIMPLE_BLINKY_DEMO_ONLY */
}
/*-----------------------------------------------------------*/




