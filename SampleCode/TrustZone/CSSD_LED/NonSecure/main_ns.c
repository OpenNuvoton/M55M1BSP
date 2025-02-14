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
#include "cssd_nsclib.h"    /* Collaborative Secure Software Development Library header */

#define NON_SECURE_LED1  PH4_NS     /* NuMaker LED_R */

void NonSecure_LED(uint32_t u32Num, uint32_t bOn);
void SysTick_Handler(void);

/*---------------------------------------------------------------------------
 * Non-secure Functions from Non-secure Region
 *---------------------------------------------------------------------------*/
void NonSecure_LED1(uint32_t u32Num, uint32_t bOn)
{
    (void)u32Num;
    printf("Non-secure LED %s call by Non-secure\n", (bOn == LED_ON) ? "On" : "Off");
    NON_SECURE_LED1 = bOn;
}

/*---------------------------------------------------------------------------
 * Non-secure SysTick IRQ Handler of second developer
 *---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    switch (u32Ticks++)
    {
        case   0:
            NonSecure_LED1(7u, LED_ON);
            Secure_LED1(0u, LED_ON);
            break;

        case 100:
            Secure_LED1(0u, LED_OFF);
            Secure_LED2(0u, LED_ON);
            break;

        case 200:
            Secure_LED2(0u, LED_OFF);
            break;

        case 300:
            NonSecure_LED1(7u, LED_OFF);
            break;

        case 400:
            Secure_LED1(0u, LED_ON);
            Secure_LED2(0u, LED_ON);
            break;

        case 500:
            Secure_LED1(0u, LED_OFF);
            Secure_LED2(0u, LED_OFF);
            break;

        case 600:
            u32Ticks = 0;
            break;

        default:
            if (u32Ticks > 600)
            {
                u32Ticks = 0;
            }
    }
}

/*---------------------------------------------------------------------------
 * Main function
 *---------------------------------------------------------------------------*/
int main(void)
{
    printf("+-----------------------------------------+\n");
    printf("|       Non-secure code is running        |\n");
    printf("+-----------------------------------------+\n");

    /* Init GPIO Port H Pin 4 for Non-secure LED control */
    GPIO_SetMode(PH_NS, BIT4, GPIO_MODE_OUTPUT);

    /* Call Secure API to get system core clock */
    SystemCoreClock = GetSystemCoreClock();

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    /* Waiting for Secure/Non-secure SysTick interrupt */
    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
