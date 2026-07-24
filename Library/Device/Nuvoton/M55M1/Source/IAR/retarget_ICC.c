/**************************************************************************//**
 * @file     retarget_ICC.c
 * @version  V1.00
 * @brief    Debug Port and Semihost Setting Source File for ICC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <LowLevelIOInterface.h>
#include "NuMicro.h"

/* Add dummy definitions to suppress MISRA-C config error.
 * (Copied from IAR installation folder/arm/inc/c/LowLevelIOInterface.h)
 */
#ifndef _LLIO_STDIN
    #define _LLIO_STDIN     0
#endif

#ifndef _LLIO_STDOUT
    #define _LLIO_STDOUT    1
#endif

#ifndef _LLIO_STDERR
    #define _LLIO_STDERR    2
#endif

#ifndef STDIN_ECHO
    #define STDIN_ECHO      0
#endif

size_t __write(int handle, const unsigned char *buffer, size_t size);
size_t __read(int handle, unsigned char *buffer, size_t size);
long __lseek(int handle, long offset, int whence);
void __exit(int return_code);
int __close(int handle);
int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);

/* To suppress misra-c2012-21.2 violation. */
extern int remove(const char *filename);

size_t __write(int handle, const unsigned char *buffer, size_t size)
{
    size_t nChars            = 0U;
    const unsigned char *buf = buffer;
    size_t len               = size;

    if (buf == (const unsigned char *)NULL)
    {
        /*
         * This means that we should flush internal buffers.  Since we
         * don't we just return.  (Remember, "handle" == -1 means that all
         * handles should be flushed.)
         */
        return 0U;
    }

    /* This template only writes to "standard out" and "standard err",
     * for all other file handles it returns failure. */
    if ((handle != _LLIO_STDOUT) && (handle != _LLIO_STDERR))
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; len != 0U; --len)
    {
        stdout_putchar(*buf);
        buf++;
        ++nChars;
    }

    return nChars;
}

size_t __read(int handle, unsigned char *buffer, size_t size)
{
    /* Remove the #if #endif pair to enable the implementation */
    size_t nChars      = 0U;
    unsigned char *buf = buffer;
    size_t len         = size;

    /* This template only reads from "standard in", for all other file
     * handles it returns failure. */
    if (handle != _LLIO_STDIN)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; len > 0U; --len)
    {
        int c = stdin_getchar();

        if (c & 0x80)
        {
            // Discard non-ASCII value
            break;
        }

#if (STDIN_ECHO != 0)
        stdout_putchar(c);
#endif

        *buf = (unsigned char)c;
        buf++;
        ++nChars;
    }

    return nChars;
}

long __lseek(int handle, long offset, int whence)
{
    (void)handle;
    (void)offset;
    (void)whence;
    return -1L;
}

#ifdef DEBUG_ENABLE_SEMIHOST
/**
 * @brief  Check if debug message finished
 *
 * @return   1 Message is finished.
 *           0 Message is transmitting.
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */
int IsDebugFifoEmpty(void)
{
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0U);
}

void __exit(int return_code)
{
    (void)return_code;

    /* Check if link with ICE */
    if (SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0) {}
    }

    for (;;) {}  /* Endless loop */
}
#else
void __exit(int return_code)
{
    char exit_code_buffer[32] = {0};
    const char *p             = exit_code_buffer;

    // Print out the exit code on the uart so any reader know how we exit.
    (void)snprintf(exit_code_buffer,
                   sizeof(exit_code_buffer),
                   "Exit code: %d.\n"      // Let the readers know how we exit
                   "\04\n",                // end-of-transmission
                   return_code);

    while (*p != '\0')
    {
        stdout_putchar(*p++);
    }

    for (;;) {}
}
#endif

int __close(int handle)
{
    (void)handle;
    return 0;
}

int remove(const char *filename)
{
    (void)filename;
    return 0;
}
