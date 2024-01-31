/**************************************************************************//**
 * @file    main_ns.c
 * @version V1.00
 * @brief   Non-secure sample code for TrustZone
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"    /* Device header */
#include "nsclib.h"     /* Collaborative Secure Software Development Library header */

typedef int32_t (*PFN_APP_ENTRY)(uint32_t);

/*---------------------------------------------------------------------------
 * Non-secure function used for Secure callback
 *---------------------------------------------------------------------------*/
int32_t NonSecure_LED_On(uint32_t u32Ticks)
{
    printf("[%d] Non-secure LED On call by Secure\n", u32Ticks);
    PC0_NS = 0;

    return 0;
}

int32_t NonSecure_LED_Off(uint32_t u32Ticks)
{
    printf("[%d] Non-secure LED Off call by Secure\n", u32Ticks);
    PC0_NS = 1;

    return 0;
}

/*---------------------------------------------------------------------------
 * Non-secure LED control
 *---------------------------------------------------------------------------*/
void LED_On(uint32_t u32Ticks)
{
    printf("[%d] Non-secure LED On\n", u32Ticks);
    PC1_NS = 0;
}

void LED_Off(uint32_t u32Ticks)
{
    printf("[%d] Non-secure LED Off\n", u32Ticks);
    PC1_NS = 1;
}

/*---------------------------------------------------------------------------
 * Non-secure SysTick IRQ Handler
 *---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    switch (u32Ticks++)
    {
        case   0:
            LED_On(u32Ticks);
            break;

        case 200:
            Secure_LED_On(u32Ticks);
            break;

        case 300:
            LED_Off(u32Ticks);
            break;

        case 500:
            Secure_LED_Off(u32Ticks);
            break;

        default:
            if (u32Ticks > 600)
            {
                u32Ticks = 0;
            }
    }
}

void DEBUG_PORT_Init(void)
{
    /* Init UART to 115200-8n1 for print message */
    DEBUG_PORT->LINE = (UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
    DEBUG_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200));
}

/*---------------------------------------------------------------------------
 * Main function
 *---------------------------------------------------------------------------*/
int main(void)
{
    DEBUG_PORT_Init();

    printf("+-----------------------------------------+\n");
    printf("|       Non-secure code is running        |\n");
    printf("+-----------------------------------------+\n");

    SecureFunc();

    /* Init GPIO Port C Pin 0 & 1 for Non-secure LED control */
    GPIO_SetMode(PC, (BIT1 | BIT0), GPIO_MODE_OUTPUT);

    /* Register Non-secure callbacks in Secure application */
    Secure_LED_On_Callback(NonSecure_LED_On);
    Secure_LED_Off_Callback(NonSecure_LED_Off);

    /* Generate Systick interrupt each 20 ms */
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 50);

    while (1);
}

void Boot_App(uint32_t u32AppBase)
{
    PFN_APP_ENTRY pfnAppEntry;
    uint32_t u32StackBase;

    /* SCB.VTOR points to the Non-secure vector table base address. */
    SCB->VTOR = u32AppBase;

    /* 1st entry in the vector table is the Non-secure Main Stack Pointer. */
    __set_MSP(*((uint32_t *)SCB->VTOR));      /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    pfnAppEntry = ((PFN_APP_ENTRY)(*(((uint32_t *)SCB->VTOR) + 1)));

    /* Check if the stack is in Secure SRAM space */
    u32StackBase = M32(u32AppBase);

    if ((u32StackBase >= NON_SECURE_SRAM_BASE) && (u32StackBase < ((NS_OFFSET + SRAM2_BASE + SRAM2_SIZE) - NON_SECURE_SRAM_BASE)))
    {
        printf("Execute App code ...\n");
        pfnAppEntry(0);
    }
    else
    {
        /* Something went wrong */
        printf("No code in App region !\n");

        while (1);
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
