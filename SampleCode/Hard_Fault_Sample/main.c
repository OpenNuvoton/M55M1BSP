/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Show hard fault information when hard fault happened.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"

/*
 The ARM Generic User Guide lists the following sources for a hard fault:

 All faults result in the HardFault exception being taken or cause lockup if
 they occur in the NMI or HardFault handler. The faults are:
  - execution of an SVC instruction at a priority equal or higher than SVCall
  - execution of a BKPT instruction without a debugger attached
  - a system-generated bus error on a load or store
  - execution of an instruction from an XN memory address
  - execution of an instruction from a location for which the system generates a bus fault
  - a system-generated bus error on a vector fetch execution of an Undefined instruction
  - execution of an instruction when not in Thumb-State as a result of the T-bit being previously cleared to 0
  - an attempted load or store to an unaligned address.

 In this sample code, we will show you some information to debug with hardfault exception.
 if HardFault_Handler is not be implemented, hardfault exception will jump to ProcessHardFault.

  - Default Process Hard Fault Handler

    The default hard fault handler called ProcessHardFault is implemented in retarget.c.
    By default, ProcessHardFault will print out the information of r0, r1, r2, r3, r12, lr, pc and psr.

  - Overwrite the default Process Hard Fault Handler

    The default ProcessHardFault is a weak function.
    User can over write it by implementing their own ProcessHardFault.

 NOTE:
    Because hard fault exception will cause data stacking.
    User must make sure SP is pointing to an valid memory location.
    Otherwise, it may cause system lockup and reset when hard fault.
*/

/* Select to use default process hard fault handler or not. 0: Default  1: User defined */
#define USE_MY_HARDFAULT    1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

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

    /* Enable module clock */
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_TMRSEL_TMR1SEL_HIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

#if USE_MY_HARDFAULT
/**
  * @brief      User defined Process HardFault
  * @param      stack   A pointer to current stack
  * @return     None
  * @details    This function is an example to show how to implement user's process hard fault handler
  *
  */
void ProcessHardFault(uint32_t *pu32StackFrame)
{
    uint32_t u32ExceptionNum;
    uint32_t u32R0, u32R1, u32R2, u32R3, u32R12, u32LR, u32PC, u32PSR;

    /* Get information from stack */
    u32R0  = pu32StackFrame[0];
    u32R1  = pu32StackFrame[1];
    u32R2  = pu32StackFrame[2];
    u32R3  = pu32StackFrame[3];
    u32R12 = pu32StackFrame[4];
    u32LR  = pu32StackFrame[5];
    u32PC  = pu32StackFrame[6];
    u32PSR = pu32StackFrame[7];

    /* Check T bit of psr */
    if ((u32PSR & (1 << 24)) == 0)
    {
        printf("PSR T bit is 0.\nHard fault caused by changing to ARM mode!\n");

        while (1);
    }

    /* Check hard fault caused by ISR */
    u32ExceptionNum = u32PSR & xPSR_ISR_Msk;

    if (u32ExceptionNum > 0)
    {
        /*
        Exception number
        0    = Thread mode
        1    = Reset
        2    = NMI
        3    = HardFault
        4    = MemManage
        5    = BusFault
        6    = UsageFault
        7    = SecureFault
        8-10 = Reserved
        11   = SVCall
        12   = DebugMonitor
        13   = Reserved
        14   = PendSV
        15   = SysTick
        16   = IRQ0.
        ...
        183  = IRQ167
        */

        printf("Hard fault is caused in IRQ #%u\n", u32ExceptionNum - 16);

        while (1);
    }

    printf("Hard fault location is at 0x%08x\n", u32PC);
    /*
        If the hard fault location is a memory access instruction, You may debug the load/store issues.

        Memory access faults can be caused by:
            Invalid address - To read/write wrong address
            Data alignment issue - Violate alignment rule of Cortex-M processor
            Memory access permission - MPU violations or unprivileged access
            Bus components or peripheral returned an error response.
    */

    printf("r0  = 0x%x\n", u32R0);
    printf("r1  = 0x%x\n", u32R1);
    printf("r2  = 0x%x\n", u32R2);
    printf("r3  = 0x%x\n", u32R3);
    printf("r12 = 0x%x\n", u32R12);
    printf("lr  = 0x%x\n", u32LR);
    printf("pc  = 0x%x\n", u32PC);
    printf("psr = 0x%x\n", u32PSR);

    while (1);
}
#endif  // USE_MY_HARDFAULT

NVT_ITCM void TIMER1_IRQHandler(void)
{
    printf("This is exception n = %d\n", TIMER1_IRQn);
    M32(0xFFFFFFFF) = 0;
    TIMER_ClearIntFlag(TIMER1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    void (*func)(void) = (void (*)(void))0x1000;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    while (1)
    {
        printf("\n\n");
        printf("+----------------------------------------------------+\n");
        printf("|        Hard Fault Handler Sample Code              |\n");
        printf("+----------------------------------------------------+\n");
        printf("| [0] Test Load/Store Hard Fault                     |\n");
        printf("| [1] Test Thumb/ARM mode Hard Fault                 |\n");
        printf("| [2] Test Hard Fault in ISR                         |\n");
        printf("+----------------------------------------------------+\n");
        char i8ch;
        i8ch = getchar();

        switch (i8ch)
        {
            case '0':
                /* Write APROM will cause hard fault exception. (Memory access hard fault) */
                M32(FMC_APROM_BASE) = 0;
                break;

            case '1':
                /* Call function with bit0 = 0 will cause hard fault. (Change to ARM mode hard fault) */
                func();
                break;

            case '2':
                /* Generate Timer Interrupt to test hard fault in ISR */
                TIMER_Open(TIMER1, TIMER_ONESHOT_MODE, 1000);
                TIMER_EnableInt(TIMER1);
                NVIC_EnableIRQ(TIMER1_IRQn);
                TIMER_Start(TIMER1);
                break;

            default:
                break;
        }

    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
