/**************************************************************************//**
 * @file     retarget_ARMCC.c
 * @version  V1.00
 * @brief    Debug Port and Semihost Setting Source File for ARMCC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <rt_misc.h>
#include <rt_sys.h>


/* Standard IO device handles. */
#define STDIN  0x8001
#define STDOUT 0x8002
#define STDERR 0x8003

#define RETARGET(fun)  _sys##fun
#define IO_OUTPUT(len) 0


/* Standard IO device name defines. */
const __ALIGNED(4) char __stdin_name [] = "STDIN";
const __ALIGNED(4) char __stdout_name[] = "STDOUT";
const __ALIGNED(4) char __stderr_name[] = "STDERR";

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
                SendChar(buf[i]);
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

    memset(buf, 0, len);

    switch (fh)
    {
        case STDIN:
        {
            int c;
            unsigned int i;

            for (i = 0; i < len; i++)
            {
                c = GetChar();

                if (c == EOF)
                {
                    return EOF;
                }

                buf[i] = (unsigned char)c;
#if (STDIN_ECHO != 0)
                SendChar(c);
#endif

                if (c == '\r')
                {
                    buf[i] = '\n';
                    i++;
                    break;
                }

                if (c == '\n')
                {
                    i++;
                    break;
                }

            }

            return (len - i);
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
    SendChar(ch);
    return;
}


#ifdef __MICROLIB
__attribute__((weak))
void abort(void)
{
    for (;;);
}

__attribute__((weak, noreturn))
void __aeabi_assert(const char *expr, const char *file, int line)
{
    char str[12], *p;

    fputs("*** assertion failed: ", stderr);
    fputs(expr, stderr);
    fputs(", file ", stderr);
    fputs(file, stderr);
    fputs(", line ", stderr);

    p = str + sizeof(str);
    *--p = '\0';
    *--p = '\n';

    while (line > 0)
    {
        *--p = '0' + (line % 10);
        line /= 10;
    }

    fputs(p, stderr);

    abort();

    while (1) {}
}

__attribute__((weak))
int fputc(int ch, FILE *f)
{
    SendChar(ch);
    return ch;
}

__attribute__((weak))
int fgetc(FILE *f)
{
    char ch = GetChar();

#if (STDIN_ECHO != 0)
    SendChar(ch);
#endif
    return (int)ch;
}

#endif
