/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Secure sample code for HardFault
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"                    /* Device header */
#include "partition_M55M1.h"

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

#define LOOP_HERE       0xE7FEE7FF      /* Instruction Code of "B ." */

/* typedef for Non-secure callback functions */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void);

/*----------------------------------------------------------------------------
 * Secure functions exported to Non-secure application
 * Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void)
{
    printf("System core clock = %d.\n", SystemCoreClock);

    return SystemCoreClock;
}

void HardFault_Handler(void)
{
    PB0 = 0;
    printf("Secure Hard Fault Handler\n");

    while (1);
}

void SYS_Init(void);
void Boot_NonSecure(uint32_t u32NonSecureBase);

/*----------------------------------------------------------------------------
 * Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    char ch;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("+-----------------------------------------+\n");
    printf("|          Secure code is running         |\n");
    printf("+-----------------------------------------+\n");

    SYS_UnlockReg();
    FMC_Open();
    /* Check Secure/Non-secure base address configuration */
    printf("SCU->FNSADDR: 0x%08X, NSCBA:        0x%08X\n", SCU->FNSADDR, FMC_Read(FMC_NSCBA_BASE));
    printf("SRAM0MPCLUT0: 0x%08X\n", SCU->SRAM0MPCLUT0);
    printf("SRAM1MPCLUT0: 0x%08X\n", SCU->SRAM1MPCLUT0);
    printf("SRAM2MPCLUT0: 0x%08X\n", SCU->SRAM2MPCLUT0);

    /* Init GPIO Port A Pin 10 & 11 for Non-secure LED control */
    GPIO_SetMode(PA_NS, (BIT10 | BIT11), GPIO_MODE_OUTPUT);
    PA10_NS = 0;
    PA11_NS = 1;

#if 0
    // Set Hard Fault to Non-secure
    SCB->AIRCR = (0x05FA << SCB_AIRCR_VECTKEY_Pos) | ((SCB->AIRCR & 0xFFFF) | SCB_AIRCR_BFHFNMINS_Msk);
#else   // Set Hard Fault to Secure
    SCB->AIRCR = (0x05FA << SCB_AIRCR_VECTKEY_Pos) | ((SCB->AIRCR & 0xFFFF) & ~SCB_AIRCR_BFHFNMINS_Msk);
#endif

    /*
        Warning: It is not recommended to set Hard Fault handler to be Non-secure.
        By default, the Hard Fault handler should be Secure.
    */
    if (SCB->AIRCR & SCB_AIRCR_BFHFNMINS_Msk)
    {
        printf(" * Non-secure Hard Fault is handled by \"Non-secure code\".\n");
    }
    else
    {
        printf(" * Non-secure Hard Fault is handled by \"Secure code\".\n");
    }

    do
    {
        printf("+------------------------------------------------------+\n");
        printf("| [0] Go Non-secure code                               |\n");
        printf("| [1] Write 0x%08X to generate Hard Fault          |\n", (uint32_t)FMC_APROM_BASE);
        printf("| [2] Write 0x%08X to generate Hard Fault          |\n", (uint32_t)FMC_NON_SECURE_BASE);
        printf("| [3] Write 0 to 0x20050000 to generate Hard Fault     |\n");
        printf("+------------------------------------------------------+\n");
        ch = (char)getchar();

        switch (ch)
        {
            case '0':
                PA10_NS = 1;
                Boot_NonSecure(FMC_NON_SECURE_BASE);
                break;

            case '1':
                M32(FMC_APROM_BASE) = 0;        /* Write to APROM generate Hard Fault */
                break;

            case '2':
                M32(FMC_NON_SECURE_BASE) = 0;   /* Write to APROM generate Hard Fault */
                break;

            case '3':
                M32(0x20050000) = 0;            /* Write to invalid address generate Hard Fault */
                break;

            default:
                break;
        }
    }

    while (1);
}

/*---------------------------------------------------------------------------
 * Boot_NonSecure function is used to jump to Non-secure boot code.
 *---------------------------------------------------------------------------*/
void Boot_NonSecure(uint32_t u32NonSecureBase)
{
    PFN_NON_SECURE_FUNC pfnNonSecureEntry;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = u32NonSecureBase;

    /* 1st entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    pfnNonSecureEntry = ((PFN_NON_SECURE_FUNC)(*(((uint32_t *)SCB_NS->VTOR) + 1)));

    /* Clear the LSB of the function address to indicate the function-call
       will cause a state switch from Secure to Non-secure */
    pfnNonSecureEntry = cmse_nsfptr_create(pfnNonSecureEntry);

    /* Check if the Reset_Handler address is in Non-secure space */
    if (cmse_is_nsfptr(pfnNonSecureEntry) && (((uint32_t)pfnNonSecureEntry & 0xF0000000) == NS_OFFSET))
    {
        printf("Execute Non-secure code ...\n");
        pfnNonSecureEntry(0);   /* Non-secure function entry */
    }
    else
    {
        /* Something went wrong */
        printf("No code in Non-secure region !\n");
        printf("CPU will halted at Non-secure state.\n");

        /* Set a temporary Non-secure MSP in Non-secure region here.
           Normally it should set to  1st entry in the vector table of Non-secure fw. */
        __TZ_set_MSP_NS(NON_SECURE_SRAM_BASE + 512);

        /* Try to halted in Non-secure state (SRAM) */
        M32(NON_SECURE_SRAM_BASE) = LOOP_HERE;
        pfnNonSecureEntry = (PFN_NON_SECURE_FUNC)(NON_SECURE_SRAM_BASE + 1);
        pfnNonSecureEntry(0);

        while (1);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*-----------------------------------------------------------------------
     * Init System Clock
     *-----------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);

    /*-----------------------------------------------------------------------
     * Init I/O Multi-function
     *-----------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
