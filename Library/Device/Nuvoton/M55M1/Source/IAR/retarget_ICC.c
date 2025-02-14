/**************************************************************************//**
 * @file     retarget_ICC.c
 * @version  V1.00
 * @brief    Debug Port and Semihost Setting Source File for ICC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <LowLevelIOInterface.h>


size_t __write(int handle, const unsigned char *buffer, size_t size)
{
    /* Remove the #if #endif pair to enable the implementation */

    size_t nChars = 0;

    if (buffer == 0)
    {
        /*
         * This means that we should flush internal buffers.  Since we
         * don't we just return.  (Remember, "handle" == -1 means that all
         * handles should be flushed.)
         */
        return 0;
    }

    /* This template only writes to "standard out" and "standard err",
     * for all other file handles it returns failure. */
    if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size != 0; --size)
    {
        SendChar(*buffer++);
        ++nChars;
    }

    return nChars;
}

size_t __read(int handle, unsigned char *buffer, size_t size)
{
    /* Remove the #if #endif pair to enable the implementation */
    int nChars = 0;

    /* This template only reads from "standard in", for all other file
     * handles it returns failure. */
    if (handle != _LLIO_STDIN)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size > 0; --size)
    {
        int c = GetChar();

        if (c < 0)
            break;

#if (STDIN_ECHO != 0)
        SendChar(c);
#endif

        *buffer++ = c;
        ++nChars;
    }

    return nChars;
}

long __lseek(int handle, long offset, int whence)
{
    return -1;
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
    /* Check if link with ICE */
    if (SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0);
    }

label:
    goto label;  /* Endless loop */
}
#else
void __exit(int return_code)
{
    char exit_code_buffer[32] = {0};
    const char *p             = exit_code_buffer;

    // Print out the exit code on the uart so any reader know how we exit.
    snprintf(exit_code_buffer,
             sizeof(exit_code_buffer),
             "Exit code: %d.\n"      // Let the readers know how we exit
             "\04\n",                // end-of-transmission
             return_code);

    while (*p != '\0')
    {
        SendChar(*p++);
    }

    while (1) {}
}
#endif

int __close(int handle)
{
    return 0;
}

int remove(const char *filename)
{
    return 0;
}
