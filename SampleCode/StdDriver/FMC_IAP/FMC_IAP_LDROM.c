/******************************************************************************
* @file    main.c
* @version V1.00
* @brief   IAP boot from LDROM sample code
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
/* Empty function to reduce code size for GCC */
void ProcessHardFault(uint32_t *pu32StackFrame) {}
#endif

typedef void (*PFN_FUNC_PTR)(void);

/* Simplified UART functions to reduce code size */
/**
 * @brief       Routine to send a char
 * @param[in]   ch Character to send to debug port.
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(char cChar)
{
    while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    DEBUG_PORT->DAT = cChar;

    if (cChar == '\n')
    {
        while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

        DEBUG_PORT->DAT = '\r';
    }
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while (1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (DEBUG_PORT->DAT);
        }
    }
}

static void PutString(char *pcStr)
{
    while (*pcStr != '\0')
    {
        SendChar_ToUART(*pcStr++);
    }
}

static void SYS_Init(void)
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

    /* Enable ISP module clock */
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    PFN_FUNC_PTR pfnAPROM_Entry;    /* function pointer */
    uint32_t     u32TimeOutCnt;     /* time-out counter */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    PutString("\n\n");
    PutString("+-------------------------------------+\n");
    PutString("|     M55M1 FMC IAP Sample Code       |\n");
    PutString("|          [LDROM code]               |\n");
    PutString("+-------------------------------------+\n");

    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */

    PutString("\n\nPress any key to branch to APROM...\n");
    GetChar();                         /* block on waiting for any one character input from UART */

    PutString("\n\nChange VECMAP and branch to APROM...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk))        /* wait until UART TX FIFO is empty */
        if (--u32TimeOutCnt == 0) break;

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);        /* Vector remap APROM page 0 to address 0. */

    if (g_FMC_i32ErrCode != 0)
    {
        PutString("FMC_SetVectorPageAddr(FMC_APROM_BASE) failed!\n");
        goto lexit;
    }

    SYS_LockReg();                                /* Lock protected registers */

    /*
     *  The reset handler address of an executable image is located at offset 0x4.
     *  Thus, this sample get reset handler address of APROM code from FMC_APROM_BASE + 0x4.
     */
    pfnAPROM_Entry = (PFN_FUNC_PTR)(M32(FMC_APROM_BASE + 4));

    /*
     *  The stack base address of an executable image is located at offset 0x0.
     *  Thus, this sample get stack base address of APROM code from FMC_APROM_BASE + 0x0.
     */

    __set_MSP(M32(FMC_APROM_BASE));

    /*
     *  Branch to the APROM code's reset handler in way of function call.
     */
    pfnAPROM_Entry();

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
