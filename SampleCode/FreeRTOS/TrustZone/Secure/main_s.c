/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Secure sample code for Collaborative Secure Software Development
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"                    /* Device header */
#include "partition_M55M1.h"

#define LOOP_HERE               0xE7FEE7FF      /* Instruction Code of "B ." */
#define INVALID_NON_SECURE_ENTERY_MSP   (NON_SECURE_SRAM_BASE + 512)    /* Temporary NS MSP for invalid NS entry */

/* typedef for Non-secure callback functions */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

void SYS_Init(void);
void Boot_NonSecure(uint32_t u32NonSecureBase);

void InitNSDebugUart(void)
{
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1_NS, 115200);
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
    InitNSDebugUart();

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
    //printf("SRAM0MPCLUT0: 0x%08X, SRAM1MPCLUT0: 0x%08X\n", SCU->SRAM0MPCLUT0, SCU->SRAM1MPCLUT0);
    printf("SRAM0MPCLUT0: 0x%08X\n", SCU->SRAM0MPCLUT0);
    printf("SRAM1MPCLUT0: 0x%08X\n", SCU->SRAM1MPCLUT0);
    printf("SRAM2MPCLUT0: 0x%08X\n", SCU->SRAM2MPCLUT0);

    /* Init GPIO Port A Pin 10 ~ 13 for Secure LED control */
    GPIO_SetMode(PA, (BIT10 | BIT11 | BIT12 | BIT13), GPIO_MODE_OUTPUT);

    /* Init GPIO Port C Pin 1 for Non-secure LED control */
    GPIO_SetMode(PC_NS, BIT1, GPIO_MODE_OUTPUT);

    /* Init and jump to Non-secure code */
    Boot_NonSecure(FMC_NON_SECURE_BASE);

    do
    {
    } while (1);
}

/*---------------------------------------------------------------------------
 *  Boot_NonSecure function is used to jump to Non-secure boot code.
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

        /* Set Non-secure MSP in Non-secure region */
        __TZ_set_MSP_NS(INVALID_NON_SECURE_ENTERY_MSP);

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
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);

    /*-----------------------------------------------------------------------
     * Init I/O Multi-function
     *-----------------------------------------------------------------------*/
    SetDebugUartMFP();

    /*----------------------------------------------------------------------*/
    /* Enable Non-Secure IP/GPIO clock                                      */
    /* (Config UART1 to Non-Secure in partition_M55M1.h)          */
    /*----------------------------------------------------------------------*/
    /* Enable UART1 module clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_UARTSEL0_UART1SEL_HXT, CLK_UARTDIV0_UART1DIV(1));
    SET_UART1_RXD_PB2();
    SET_UART1_TXD_PB3();

    /* Lock protected registers */
    SYS_LockReg();
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
