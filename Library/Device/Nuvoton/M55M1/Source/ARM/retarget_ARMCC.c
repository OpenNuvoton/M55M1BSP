/**************************************************************************//**
 * @file     retarget_ARMCC.c
 * @version  V1.00
 * @brief    Debug Port and Semihost Setting Source File for ARMCC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#if defined (__ARMCC_VERSION)

#include <rt_misc.h>
#include <rt_sys.h>
#include "NuMicro.h"

/* Add dummy definition to suppress cppcheck unknownMacro error. */
#ifndef __ALIGNED
    #define __ALIGNED(x)    __attribute__((aligned(x)))
#endif

#ifndef   __WEAK
    #define __WEAK          __attribute__((weak))
#endif

#ifndef STDIN_ECHO
    #define STDIN_ECHO      0
#endif

/* Standard IO device handles. */
#define STDIN               0x8001
#define STDOUT              0x8002
#define STDERR              0x8003

#define RETARGET(fun)       _sys##fun
#define IO_OUTPUT(len)      0

/* Standard IO device name defines. */
__WEAK __ALIGNED(4) const char __stdin_name [] = "STDIN";
__WEAK __ALIGNED(4) const char __stdout_name[] = "STDOUT";
__WEAK __ALIGNED(4) const char __stderr_name[] = "STDERR";

FILEHANDLE RETARGET(_open)(const char *name, int openmode)
{
    (void)openmode;

    if (strcmp(name, __stdin_name) == 0)
    {
        return (STDIN);
    }

    if (strcmp(name, __stdout_name) == 0)
    {
        return (STDOUT);
    }

    if (strcmp(name, __stderr_name) == 0)
    {
        return (STDERR);
    }

    return -1;
}

int RETARGET(_write)(FILEHANDLE fh, const unsigned char *buf, unsigned int len, int mode)
{
    (void)mode;

    switch (fh)
    {
        case STDOUT:
        case STDERR:
        {
            unsigned int i;

            for (i = 0; i < len; i++)
            {
                stdout_putchar(buf[i]);
            }

            return IO_OUTPUT(len);
        }

        default:
            return EOF;
    }
}

int RETARGET(_read)(FILEHANDLE fh, unsigned char *buf, unsigned int len, int mode)
{
    (void)mode;

    (void)memset(buf, 0, len);

    switch (fh)
    {
        case STDIN:
        {
            int c;
            unsigned int i = 0U;
            unsigned int remaining;
            int done = 0;

            while ((i < len) && (done == 0))
            {
                c = stdin_getchar();

                if (c == EOF)
                {
                    return EOF;
                }

                buf[i] = (unsigned char)c;
#if (STDIN_ECHO != 0)
                stdout_putchar(c);
#endif

                if ((char)c == '\r')
                {
                    buf[i] = '\n';
                    done = 1;
                }
                else if ((char)c == '\n')
                {
                    done = 1;
                }
                else
                {
                    /* normal character */
                }

                i++;
            }

            remaining = len - i;
            return (int)remaining;
        }

        default:
            return EOF;
    }
}

int RETARGET(_istty)(FILEHANDLE fh)
{
    switch (fh)
    {
        case STDIN:
        case STDOUT:
        case STDERR:
            return 1;

        default:
            return 0;
    }
}

int RETARGET(_close)(FILEHANDLE fh)
{
    if (RETARGET(_istty(fh)))
    {
        return 0;
    }

    return -1;
}

int RETARGET(_seek)(FILEHANDLE fh, long pos)
{
    (void)fh;
    (void)pos;

    return -1;
}

int RETARGET(_ensure)(FILEHANDLE fh)
{
    (void)fh;

    return -1;
}

long RETARGET(_flen)(FILEHANDLE fh)
{
    if (RETARGET(_istty)(fh))
    {
        return 0;
    }

    return -1;
}

#if __ARMCC_VERSION >= 6190000
void RETARGET(_tmpnam)(char *name, int sig, unsigned maxlen)
{
    (void)name;
    (void)sig;
    (void)maxlen;
}
#else   /* __ARMCC_VERSION < 6190000 */
int RETARGET(_tmpnam)(char *name, int sig, unsigned maxlen)
{
    (void)name;
    (void)sig;
    (void)maxlen;

    return 1;
}
#endif

char *RETARGET(_command_string)(char *cmd, int len)
{
    (void)len;

    return cmd;
}

void RETARGET(_exit)(int return_code)
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

/**
 * @brief     C library retargetting
 *            This function writes a character to the console.
 *
 * @param[in] ch: Write a character data
 *
 * @returns   None
 */

void _ttywrch(int ch)
{
    stdout_putchar(ch);
    return;
}


#ifdef __MICROLIB
__attribute__((weak))
void abort(void)
{
    for (;;) {}
}

__attribute__((weak, noreturn))
void __aeabi_assert(const char *expr, const char *file, int line)
{
    char   str[12];
    size_t idx;
    int    line_num = line;

    fputs("*** assertion failed: ", stderr);
    fputs(expr, stderr);
    fputs(", file ", stderr);
    fputs(file, stderr);
    fputs(", line ", stderr);

    idx = sizeof(str) - 1U;
    str[idx] = '\0';
    idx--;
    str[idx] = '\n';

    while (line_num > 0)
    {
        idx--;
        str[idx] = '0' + (line_num % 10);
        line_num /= 10;
    }

    fputs(&str[idx], stderr);

    for (;;) {}
}

__attribute__((weak))
int fputc(int ch, FILE *f)
{
    (void)f;

    stdout_putchar(ch);
    return ch;
}

__attribute__((weak))
int fgetc(FILE *f)
{
    char ch = stdin_getchar();

    (void)f;

#if (STDIN_ECHO != 0)
    stdout_putchar(ch);
#endif
    return (int)ch;
}

#endif  // __MICROLIB
#endif  // defined (__ARMCC_VERSION)
