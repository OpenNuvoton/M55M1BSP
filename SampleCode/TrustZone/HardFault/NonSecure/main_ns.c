/**************************************************************************//**
 * @file    main_ns.c
 * @version V1.00
 * @brief   Non-secure sample code for HardFault
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"    /* Device header */
#include "nsclib.h"     /* Collaborative Secure Software Development Library header */

/*---------------------------------------------------------------------------
 * Non-secure Hard Fault can be enabled by Secure code through AIRCR.BFHFNMINS[13].
 *   https://developer.arm.com/documentation/ka004627
 *   https://developer.arm.com/documentation/100691/0200/AIRCR-BFHFNMINS
 * If Secure code set it to Secure, the Non-secure Hard Fault is always handled
 * by Secure Hard Fault handler.
 *
 * If Secure code set it to Non-secure, the Non-secure Hard Fault is handled
 * by Non-secure Hard Fault handler.
 * However, if Non-secure code access Secure region and cause Hard Fault.
 * It is still handled by Secure Hard Fault handler for security.
 *---------------------------------------------------------------------------*/

void HardFault_Handler(void)
{
    PA10_NS = 0;
    printf("Non-secure Hard Fault Handler\n");

    while (1);
}

/*---------------------------------------------------------------------------
 * Main function
 *---------------------------------------------------------------------------*/
int main(void)
{
    char ch;
    int32_t i;

    printf("+-----------------------------------------+\n");
    printf("|       Non-secure code is running        |\n");
    printf("+-----------------------------------------+\n");

    /* Call Secure API to get system core clock */
    SystemCoreClock = GetSystemCoreClock();

    do
    {
        printf("+-------------------------------------------------------+\n");
        printf("| [0] Write address 0x%08X to generate Hard Fault   |\n", (uint32_t)FMC_NON_SECURE_BASE);
        printf("| [1] Write Secure I/O port to generate Hard Fault      |\n");
        printf("| [2] Read Secure SRAM 0x%08X to generate Hard Fault|\n", (uint32_t)SRAM_BASE);
        printf("| [3] Toggle Non-secure I/O                             |\n");
        printf("+-------------------------------------------------------+\n");
        ch = (char)getchar();

        switch (ch)
        {
            case '0':
                /*
                    If Non-secure code access Non-secure region and cause bus error,
                    it would cause Non-secure or Secure Hard Fault
                    depends on the setting of AIRCR.BFHFNMINS[13]. (AIRCR is Secure register)
                */
                M32(FMC_NON_SECURE_BASE) = 0;
                break;

            case '1':
                /*
                    GPIO Port B is configured as Secure port. Access it in Non-secure code
                    will cause Secure Hard Fault.
                */
                PB0 = 0;
                PB1 = 0;
                break;

            case '2':
                /*
                    SRAM_BASE is configured as Secure SRAM. Access it in Non-secure code
                    will cause Secure Hard Fault.
                */
                M32(SRAM_BASE);
                break;

            case '3':
                printf("LED blinking...\n");
                PA10_NS = 0;
                PA11_NS = 1;

                while (1)
                {
                    PA10_NS ^= 1;
                    PA11_NS ^= 1;
                    i = 0x100000;

                    while (i-- > 0);

                }

            //break;
            default:
                break;
        }
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
