/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to generate the NuBL32 and can be authenticated by NuBL2.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M55M1.h"
#include "../image.h"

/*----------------------------------------------------------------------------
  Secure function for NonSecure callbacks exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void)
{
    return SystemCoreClock;
}

#define LOOP_HERE       0xE7FEE7FF      /* Instruction Code of "B ." */

/* typedef for Non-secure callback function */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

/*---------------------------------------------------------------------------
 * Boot_NonSecure function is used to jump to Non-secure boot code.
 *---------------------------------------------------------------------------*/
void Boot_NonSecure(uint32_t u32NonSecureBase)
{
    PFN_NON_SECURE_FUNC pfnNonSecureEntry;
    printf("u32NonSecureBase: 0x%08X\n", u32NonSecureBase);
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

        /* Set Non-secure MSP in Non-secure region */
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    struct image_header *psImgHdr = (struct image_header *)FMC_NON_SECURE_BASE;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|    SecureBootDemo - NuBL32 Sample Code    |\n");
    printf("+-------------------------------------------+\n\n");

    printf("System is executing in NuBL32.\n\n");

    /* Jump to execute NuBL33 FW */
    printf("\nPress any key to execute NuBL33 ...\n");
    getchar();
    /* Init and jump to Non-secure code */
    Boot_NonSecure(FMC_NON_SECURE_BASE + psImgHdr->ih_hdr_size);

    while (1) {}
}
