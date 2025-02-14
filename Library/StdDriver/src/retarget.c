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

#ifndef STDIN_ECHO
    #define STDIN_ECHO         0    // STDIN: echo to STDOUT
#endif

#ifndef TRACK_HARDFAULT
    #define TRACK_HARDFAULT    0    // Set to 1 to enable hardfault status tracking; set to 0 to disable.
#endif

#if defined(DEBUG_ENABLE_SEMIHOST)

static volatile int32_t g_ICE_Connected = 1;
static uint8_t g_buf_len = 0;

/* The static buffer is used to speed up the semihost */
#if (NVT_DCACHE_ON == 1)
    /* This buffer could be accessed by CPU and ICE therefore place in Non-Cacheable region to prevent cache coherence issues. */
    NVT_NONCACHEABLE_INIT static char g_buf[16];
#else
    static char g_buf[16];
#endif

/**
 * @brief      The function to process semihosted command
 *
 * @param[in]  n32In_R0  : semihost register 0
 * @param[in]  n32In_R1  : semihost register 1
 * @param[out] pn32Out_R0: semihost register 0
 *
 * @retval     0: No ICE debug
 * @retval     1: ICE debug
 */
int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    (void)n32In_R1;

    if (g_ICE_Connected)
    {
        if (pn32Out_R0)
            *pn32Out_R0 = n32In_R0;

        return 1;
    }

    return 0;
}

/**
 * @brief      The function to process semihosted command
 *
 * @param[in]  n32In_R0  : semihost register 0
 * @param[in]  n32In_R1  : semihost register 1
 * @param[out] pn32Out_R0: semihost register 0
 *
 * @retval     0: No ICE debug
 * @retval     1: ICE debug
 */
#if defined (__ARMCC_VERSION)
int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);
#else
int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    __ASM volatile("BKPT   0xAB\n");     // Wait ICE or HardFault
    return SH_Return(n32In_R0, n32In_R1, pn32Out_R0);
}
#endif
#endif

/**
 * @brief     Send a character data to UART debug port.
 *
 * @param[in] ch: A character data writes to debug port
 *
 * @returns   None
 */
#ifndef NONBLOCK_PRINTF
static void SendChar_ToUART(int ch)
{
    if ((char)ch == '\n')
    {
        while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}

        DEBUG_PORT->DAT = '\r';
    }

    while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}

    DEBUG_PORT->DAT = (uint32_t)ch;
}

#else
/* Non-block implement of send char */
# define BUF_SIZE    512
static void SendChar_ToUART(int ch)
{
    static uint8_t u8Buf[BUF_SIZE] = {0};
    static int32_t i32Head = 0;
    static int32_t i32Tail = 0;
    int32_t i32Tmp;

    // Only flush the data in buffer to UART when ch == 0
    if (ch)
    {
        if (ch == '\n')
        {
            i32Tmp = i32Head + 1;

            if (i32Tmp > BUF_SIZE) i32Tmp = 0;

            if (i32Tmp != i32Tail)
            {
                u8Buf[i32Head] = '\r';
                i32Head = i32Tmp;
            }
        }

        // Push char
        i32Tmp = i32Head + 1;

        if (i32Tmp > BUF_SIZE) i32Tmp = 0;

        if (i32Tmp != i32Tail)
        {
            u8Buf[i32Head] = ch;
            i32Head = i32Tmp;
        }
    }
    else
    {
        if (i32Tail == i32Head)
            return;
    }

    // pop char
    do
    {
        i32Tmp = i32Tail + 1;

        if (i32Tmp > BUF_SIZE) i32Tmp = 0;

        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) == 0)
        {
            DEBUG_PORT->DAT = u8Buf[i32Tail];
            i32Tail = i32Tmp;
        }
        else
            break; // FIFO full
    } while (i32Tail != i32Head);
}
#endif

/**
 * @brief     Send a character data to UART debug port or semihost.
 *
 * @param[in] ch: A character data writes to debug port
 *
 * @returns   None
 */
static void SendChar(int ch)
{
#if defined(DEBUG_ENABLE_SEMIHOST)
    g_buf[g_buf_len++] = (char)ch;
    g_buf[g_buf_len] = '\0';

    if (g_buf_len + 1 >= sizeof(g_buf) || ch == '\n' || ch == '\0')
    {
        // Send the char
        if (g_ICE_Connected)
        {
            if (SH_DoCommand(0x04, (int)g_buf, NULL) != 0)
            {
                g_buf_len = 0;

                return;
            }
        }
        else
        {
# if (DEBUG_ENABLE_SEMIHOST == 1) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=1
            int i;

            for (i = 0; i < g_buf_len; i++)
                SendChar_ToUART(g_buf[i]);

            g_buf_len = 0;
# endif
        }
    }

#else
    SendChar_ToUART(ch);
#endif
}

/**
 * @brief    Wait UART debug port or semihost to input a char.
 *
 * @param    None
 *
 * @returns  Get character data from UART debug port or semihost
 */
static char GetChar(void)
{
#ifdef DEBUG_ENABLE_SEMIHOST

    if (g_ICE_Connected)
    {
        int nRet;

#if defined (__ICCARM__)

        while (SH_DoCommand(0x7, 0, &nRet) != 0)
        {
            if (nRet != 0)
                return (char)nRet;
        }

#else

        while (SH_DoCommand(0x101, 0, &nRet) != 0)
        {
            if (nRet != 0)
            {
                SH_DoCommand(0x07, 0, &nRet);
                return (char)nRet;
            }
        }

#endif
    }
    else
    {
# if (DEBUG_ENABLE_SEMIHOST == 1) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=1

        // Use debug port when ICE is not connected at semihost mode
        while (!g_ICE_Connected)
        {
            if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
            {
                return (DEBUG_PORT->DAT);
            }
        }

# endif
    }

    return (0);

#else

    while (1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            return ((char)DEBUG_PORT->DAT);
        }
    }

#endif
}

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

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
    #ifndef NVT_ISP_FUNC
        #include "../../Device/Nuvoton/M55M1/Source/GCC/retarget_GCC.c"
    #else
        // To reduce code size
    #endif
#endif

#if defined (__ARMCC_VERSION)
    #include "../../Device/Nuvoton/M55M1/Source/ARM/retarget_ARMCC.c"
#endif

#if defined (__ICCARM__)
    #include "../../Device/Nuvoton/M55M1/Source/IAR/retarget_ICC.c"
#endif
