/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Secure sample code for TrustZone
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <arm_cmse.h>
#include "NuMicro.h"

#define LOOP_HERE       0xE7FEE7FF      /* Instruction Code of "B ." */

/* typedef for Non-secure callback functions */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

/*---------------------------------------------------------------------------
 * Secure function exported to Non-secure application
 * Must decalre with __NONSECURE_ENTRY and place in Non-secure Callable region
 *---------------------------------------------------------------------------*/
__NONSECURE_ENTRY
int32_t SecureFunc(void)
{
    printf("Secure function call by Non-secure.\n");

    return 1;
}

__NONSECURE_ENTRY
int32_t Secure_LED_On(uint32_t u32Ticks)
{
    printf("[%d] Secure LED On call by Non-secure.\n", u32Ticks);
    PA10 = 0;
    PG14 = 0;

    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_LED_Off(uint32_t u32Ticks)
{
    printf("[%d] Secure LED Off call by Non-secure.\n", u32Ticks);
    PA10 = 1;
    PG14 = 1;

    return 0;
}

/*---------------------------------------------------------------------------
 * For Non-secure code to register its callback function for Secure code
 *---------------------------------------------------------------------------*/
static PFN_NON_SECURE_FUNC pfnNonSecure_LED_On  = NULL;
static PFN_NON_SECURE_FUNC pfnNonSecure_LED_Off = NULL;

__NONSECURE_ENTRY
int32_t Secure_LED_On_Callback(PFN_NON_SECURE_FUNC pfnCallback)
{
    pfnNonSecure_LED_On = (PFN_NON_SECURE_FUNC)cmse_nsfptr_create(pfnCallback);
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_LED_Off_Callback(PFN_NON_SECURE_FUNC pfnCallback)
{
    pfnNonSecure_LED_Off = (PFN_NON_SECURE_FUNC)cmse_nsfptr_create(pfnCallback);
    return 0;
}

/*---------------------------------------------------------------------------
 * Secure LED control function
 *---------------------------------------------------------------------------*/
int32_t LED_On(void)
{
    printf("Secure LED On\n");
    PA11 = 0;
    PG15 = 0;

    return 0;
}

int32_t LED_Off(void)
{
    printf("Secure LED Off\n");
    PA11 = 1;
    PG15 = 1;

    return 0;
}

/*---------------------------------------------------------------------------
 * Secure SysTick IRQ Handler
 *---------------------------------------------------------------------------*/
NVT_ITCM void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    switch (u32Ticks++)
    {
        case 100:
            LED_On();
            break;

        case 200:
            LED_Off();
            break;

        case 300:
            if (pfnNonSecure_LED_On != NULL)
            {
                pfnNonSecure_LED_On(u32Ticks);
            }

            break;

        case 500:
            if (pfnNonSecure_LED_Off != NULL)
            {
#if defined (__ARMCC_VERSION) || defined (__ICCARM__)
                /* This func call caused GCC compiler internal error - Skip in GCC */
                pfnNonSecure_LED_Off(u32Ticks);
#endif
            }

            break;

        default:
            if (u32Ticks > 600)
            {
                u32Ticks = 0;
            }
    }
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

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*-----------------------------------------------------------------------
     * Init System Clock
     *-----------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*-----------------------------------------------------------------------
     * Init I/O Multi-function
     *-----------------------------------------------------------------------*/
    SetDebugUartMFP();

    /*-----------------------------------------------------------------------
     * Init GPIOI/UART1 for Non-secure console
     * (Config GPIOI/UART1 to Non-secure in partition_M55M1.h)
     *-----------------------------------------------------------------------*/
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HIRC, CLK_UARTDIV0_UART1DIV(1));
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------
 * Main function
 *---------------------------------------------------------------------------*/
int main(void)
{
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
    printf("SCU->FNSADDR: 0x%08X, NSCBA: 0x%08X\n", SCU->FNSADDR, FMC_Read(FMC_NSCBA_BASE));
    printf("SRAM0MPCLUT0: 0x%08X\n", SCU->SRAM0MPCLUT0);
    printf("SRAM1MPCLUT0: 0x%08X\n", SCU->SRAM1MPCLUT0);
    printf("SRAM2MPCLUT0: 0x%08X\n", SCU->SRAM2MPCLUT0);

    /* Init GPIO Port A Pin 10 & 11 for Secure LED control */
    GPIO_SetMode(PA, (BIT11 | BIT10), GPIO_MODE_OUTPUT);

    /* Init GPIO Port G Pin 14 & 15 for Secure LED control */
    GPIO_SetMode(PG, (BIT14 | BIT15), GPIO_MODE_OUTPUT);

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    /* Init and jump to Non-secure code */
    Boot_NonSecure(FMC_NON_SECURE_BASE);

    do
    {
        __WFI();
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
