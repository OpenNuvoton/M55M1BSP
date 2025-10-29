/**************************************************************************//**
 * @file     retarget.c
 * @version  V1.00
 * @brief    Debug port and semihost setting source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#ifndef TRACK_HARDFAULT
    #define TRACK_HARDFAULT    0    // Set to 1 to enable hardfault status tracking; set to 0 to disable.
#endif

#if defined(TRACK_HARDFAULT) && (TRACK_HARDFAULT==1)
static void usage_fault_track(uint32_t u32CFSR)
{
    /* Usage Fault Status Register */
    printf("usage fault: ");

    if (u32CFSR & SCB_CFSR_UNDEFINSTR_Msk)
    {
        printf("UNDEFINSTR ");
    }

    if (u32CFSR & SCB_CFSR_INVSTATE_Msk)
    {
        printf("INVSTATE ");
    }

    if (u32CFSR & SCB_CFSR_INVPC_Msk)
    {
        printf("INVPC ");
    }

    if (u32CFSR & SCB_CFSR_NOCP_Msk)
    {
        printf("NOCP ");
    }

    if (u32CFSR & SCB_CFSR_UNALIGNED_Msk)
    {
        printf("UNALIGNED ");
    }

    if (u32CFSR & SCB_CFSR_DIVBYZERO_Msk)
    {
        printf("DIVBYZERO ");
    }

    printf("\n");
}

static void bus_fault_track(uint32_t u32CFSR)
{
    /* Bus Fault Status Register */
    printf("usage fault: ");

    if (u32CFSR & SCB_CFSR_IBUSERR_Msk)
    {
        printf("IBUSERR ");
    }

    if (u32CFSR & SCB_CFSR_PRECISERR_Msk)
    {
        printf("PRECISERR ");
    }

    if (u32CFSR & SCB_CFSR_IMPRECISERR_Msk)
    {
        printf("IMPRECISERR ");
    }

    if (u32CFSR & SCB_CFSR_UNSTKERR_Msk)
    {
        printf("UNSTKERR ");
    }

    if (u32CFSR & SCB_CFSR_STKERR_Msk)
    {
        printf("STKERR ");
    }

    if (u32CFSR & SCB_CFSR_BFARVALID_Msk)
    {
        printf("SCB->BFAR:%08X ", SCB->BFAR);
    }

    printf("\n");
}

static void mem_manage_fault_track(uint32_t u32CFSR)
{
    /* Memory Fault Status Register */
    printf("mem manage fault: ");

    if (u32CFSR & SCB_CFSR_IACCVIOL_Msk)
    {
        printf("IACCVIOL ");
    }

    if (u32CFSR & SCB_CFSR_DACCVIOL_Msk)
    {
        printf("DACCVIOL ");
    }

    if (u32CFSR & SCB_CFSR_MUNSTKERR_Msk)
    {
        printf("MUNSTKERR ");
    }

    if (u32CFSR & SCB_CFSR_MSTKERR_Msk)
    {
        printf("MSTKERR ");
    }

    if (u32CFSR & SCB_CFSR_MLSPERR_Msk)
    {
        printf("MLSPERR ");
    }

    if (u32CFSR & SCB_CFSR_MMARVALID_Msk)
    {
        printf("SCB->MMFAR:%08X ", SCB->MMFAR);
    }

    printf("\n");

}

static void TrackHardFault(void)
{
    if (SCB->HFSR & SCB_HFSR_VECTTBL_Msk)
    {
        printf("failed vector fetch\n");
    }

    if (SCB->HFSR & SCB_HFSR_FORCED_Msk)
    {
        uint32_t u32SCB_CFSR = SCB->CFSR;

        if (u32SCB_CFSR & SCB_CFSR_BUSFAULTSR_Msk)
        {
            bus_fault_track(u32SCB_CFSR);
        }

        if (u32SCB_CFSR & SCB_CFSR_MEMFAULTSR_Msk)
        {
            mem_manage_fault_track(u32SCB_CFSR);
        }

        if (u32SCB_CFSR & SCB_CFSR_USGFAULTSR_Msk)
        {
            usage_fault_track(u32SCB_CFSR);
        }
    }

    if (SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk)
    {
        printf("debug event\n");
    }
}
#endif

__WEAK uint32_t ProcessHardFault(uint32_t *pu32StackFrame)
{
    uint32_t inst, addr, taddr, tdata;
    uint32_t rm, rn, rt, imm5, imm8;
    uint32_t u32CFSR, u32BFAR;

    // It is caused by hardfault. Just process the hard fault.
    /*
        r0  = pu32StackFrame[0]
        r1  = pu32StackFrame[1]
        r2  = pu32StackFrame[2]
        r3  = pu32StackFrame[3]
        r12 = pu32StackFrame[4]
        lr  = pu32StackFrame[5]
        pc  = pu32StackFrame[6]
        psr = pu32StackFrame[7]
    */

    if (pu32StackFrame == NULL)
        return 0;

    /* Read volatile registers into temporary variables to fix IAR [Pa082] the order of volatile accesses is undefined */
    u32CFSR = SCB->CFSR;
    u32BFAR = SCB->BFAR;

    // Get the instruction caused the hardfault
    addr = pu32StackFrame[6];
    inst = M16(addr);

    if (inst == 0xBEAB)
    {
        //printf("Execute BKPT without ICE connected\n");
        /*
            If the instruction is 0xBEAB, it means it is caused by BKPT without ICE connected.
            We still return for output/input message to UART.
        */
#if defined(DEBUG_ENABLE_SEMIHOST)
        g_ICE_Connected = 0;        // Set a flag for ICE offline
#endif
        pu32StackFrame[6] += 2;     // Return to next instruction
        return pu32StackFrame[5];   // Keep lr in R0
    }

    /* It is casued by hardfault (Not semihost). Just process the hard fault here. */
    printf("\nHardFault Analysis:\n");
    printf("    PC  : 0x%08X, LR  : 0x%08X, XPSR : 0x%08X\n",
           pu32StackFrame[5], pu32StackFrame[6], pu32StackFrame[7]);
    printf("    CFSR: 0x%08X, BFAR: 0x%08X, MMFAR: 0x%08X\n", u32CFSR, u32BFAR, SCB->MMFAR);
    printf("Instruction code = %x\n", inst);

    if ((inst >> 12) == 5)
    {
        // 0101xx Load/store (register offset) on page C2-327 of Armv8-M ref
        rm = (inst >> 6) & 0x7;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;

        printf("LDR/STR rt=%x rm=%x rn=%x\n", rt, rm, rn);
        taddr = pu32StackFrame[rn] + pu32StackFrame[rm];
        tdata = pu32StackFrame[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);
    }
    else if ((inst >> 13) == 3)
    {
        // 011xxx    Load/store word/byte (immediate offset) on page C2-327 of Armv8-M ref
        imm5 = (inst >> 6) & 0x1f;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;

        printf("LDR/STR rt=%x rn=%x imm5=%x\n", rt, rn, imm5);
        taddr = pu32StackFrame[rn] + imm5;
        tdata = pu32StackFrame[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);
    }
    else if ((inst >> 12) == 8)
    {
        // 1000xx    Load/store halfword (immediate offset) on page C2-328
        imm5 = (inst >> 6) & 0x1f;
        rn = (inst >> 3) & 0x7;
        rt = inst & 0x7;

        printf("LDRH/STRH rt=%x rn=%x imm5=%x\n", rt, rn, imm5);
        taddr = pu32StackFrame[rn] + imm5;
        tdata = pu32StackFrame[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);
    }
    else if ((inst >> 12) == 9)
    {
        // 1001xx    Load/store (SP-relative) on page C2-328
        imm8 = inst & 0xff;
        rt = (inst >> 8) & 0x7;

        printf("LDRH/STRH rt=%x imm8=%x\n", rt, imm8);
        taddr = pu32StackFrame[6] + imm8;
        tdata = pu32StackFrame[rt];
        printf("[0x%08x] 0x%04x %s 0x%x [0x%x]\n", addr, inst,
               (inst & BIT11) ? "LDR" : "STR", tdata, taddr);
    }
    else
    {
        printf("Unexpected instruction\n");
    }

    /* Enable hardfault tracking when TRACK_HARDFAULT is set to 1. */
#if defined(TRACK_HARDFAULT) && (TRACK_HARDFAULT==1)
    TrackHardFault();
#endif

#if (CHECK_SCU_VIOLATION == 1)
    // If CHECK_SCU_VIOLATION is enabled, the code continues execution to check the SCU violation status.
    return 0;
#else

    // Halt here
    while (1);

#endif
}

void MemManage_Handler(void)
{
    // 1. Check the fault status register
    uint32_t u32MMFSR = SCB->CFSR & 0xFF; // Memory Management Fault Status Register (lower 8 bits of CFSR)

    // 2. Analyze the fault
    if (u32MMFSR & SCB_CFSR_IACCVIOL_Msk)
    {
        // Instruction Access Violation (e.g., executing from a non-executable region)
        // Log or handle the error
    }

    if (u32MMFSR & SCB_CFSR_DACCVIOL_Msk)
    {
        // Data Access Violation (e.g., accessing privileged memory in unprivileged mode)
        // Log or handle the error
        printf("* Please set Default Processor mode for Thread execution to Privileged mode in RTX_Config.h !");
    }

    if (u32MMFSR & SCB_CFSR_MUNSTKERR_Msk)
    {
        // Unstacking fault during exception return
        if (MPU->CTRL & MPU_CTRL_ENABLE_Msk)
        {
            uint32_t control = __get_CONTROL();

            if (control & CONTROL_nPRIV_Msk)
            {
                printf(" * Please set Default Processor mode for Thread execution to Privileged mode in RTX_Config.h !");
            }
        }
    }

    if (u32MMFSR & SCB_CFSR_MSTKERR_Msk)
    {
        // Stacking fault during exception entry
    }

    // 3. Get the faulting address (if available)
    if (u32MMFSR & SCB_CFSR_MMARVALID_Msk)
    {
        //uint32_t fault_address = SCB->MMFAR; // Memory Management Fault Address Register
        // Use this address for debugging
    }

    while (1); // Fallback infinite loop (for debugging)
}

int stdout_putchar(int ch)
{
    if ((char)ch == '\n')
    {
        while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}

        DEBUG_PORT->DAT = '\r';
    }

    while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}

    DEBUG_PORT->DAT = (uint32_t)ch;
    return 0;
}
