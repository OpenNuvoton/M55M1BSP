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

void LED_On(uint32_t u32Num);
void LED_Off(uint32_t u32Num);
void SysTick_Handler(void);

/*---------------------------------------------------------------------------
 * Non-secure Functions from Non-secure Region
 *---------------------------------------------------------------------------*/
void LED_On(uint32_t u32Num)
{
    (void)u32Num;
    printf("Non-secure PC0 LED On call by Non-secure\n");
    PC0_NS = 0;
}

void LED_Off(uint32_t u32Num)
{
    (void)u32Num;
    printf("Non-secure PC0 LED Off call by Non-secure\n");
    PC0_NS = 1;
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
            LED_On(7u);
            Secure_PA11_LED_On(0u);
            break;

        case 100:
            Secure_PA11_LED_Off(0u);
            Secure_PA12_LED_On(0u);
            break;

        case 200:
            Secure_PA12_LED_Off(0u);
            break;

        case 300:
            LED_Off(7u);
            break;

        case 400:
            Secure_PA11_LED_On(0u);
            Secure_PA12_LED_On(0u);
            break;

        case 500:
            Secure_PA11_LED_Off(0u);
            Secure_PA12_LED_Off(0u);
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

    /* Init GPIO Port C Pin 0 for Non-secure LED control */
    GPIO_SetMode(PC_NS, BIT0, GPIO_MODE_OUTPUT);

    /* Call Secure API to get system core clock */
    SystemCoreClock = GetSystemCoreClock();

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    /* Waiting for Secure/Non-secure SysTick interrupt */
    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
