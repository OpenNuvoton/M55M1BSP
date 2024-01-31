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

static volatile int32_t g_ICE_Connected = 1;

#ifndef STDIN_ECHO
    #define STDIN_ECHO      0       // STDIN: echo to STDOUT
#endif

#if defined(DEBUG_ENABLE_SEMIHOST)
/* The static buffer is used to speed up the semihost */
static char g_buf[16];
static uint8_t g_buf_len = 0;

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
 * @brief     Routine to send a char
 *
 * @param[in] ch  A character data writes to debug port
 * @returns   Send value from UART debug port
 *
 * @details   Send a target char to UART debug port .
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
 * @brief     Routine to send a char
 *
 * @param[in] ch A character data writes to debug port
 * @returns   Send value from UART debug port or semihost
 *
 * @details   Send a target char to UART debug port or semihost.
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
 * @brief    Routine to get a char
 *
 * @param    None
 *
 * @returns  Get value from UART debug port or semihost
 *
 * @details  Wait UART debug port or semihost to input a char.
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
        g_ICE_Connected = 0;        // Set a flag for ICE offline
        pu32StackFrame[6] += 2;     // Return to next instruction
        return pu32StackFrame[5];   // Keep lr in R0
    }

    /* It is casued by hardfault (Not semihost). Just process the hard fault here. */
    printf("HardFault Analysis:\n");
    printf("Hard fault. pc=0x%08X, lr=0x%08X, xpsr=0x%08X\n",
           pu32StackFrame[5], pu32StackFrame[6], pu32StackFrame[7]);
    printf("%11s cfsr=0x%08X, bfar=0x%08X, mmfar=0x%08X\n", "", u32CFSR, u32BFAR, SCB->MMFAR);
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

    // Halt here
    while (1);
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
