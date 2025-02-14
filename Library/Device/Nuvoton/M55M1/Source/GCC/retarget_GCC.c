/**************************************************************************//**
 * @file     retarget_GCC.c
 * @version  V1.00
 * @brief    Debug Port and Semihost Setting Source File for GCC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "NuMicro.h"


/*
 * This type is used by the _ I/O functions to denote an open
 * file.
 */
typedef int FILEHANDLE;

/*
 * Open a file. May return -1 if the file failed to open.
 */
extern FILEHANDLE _open(const char * /*name*/, int /*openmode*/);

/* Standard IO device handles. */
#define STDIN  0x00
#define STDOUT 0x01
#define STDERR 0x02

#define RETARGET(fun)  fun
#define IO_OUTPUT(len) len


/* Standard IO device name defines. */
const __ALIGNED(4) char __stdin_name [] = "STDIN";
const __ALIGNED(4) char __stdout_name[] = "STDOUT";
const __ALIGNED(4) char __stderr_name[] = "STDERR";

#if defined (OS_USE_SEMIHOSTING)

#else
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

                if ((c == '\n') || (c == '\r'))
                {
                    i++;
                    break;
                }

            }

            return i;
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

int RETARGET(_tmpnam)(char *name, int sig, unsigned maxlen)
{
    (void)name;
    (void)sig;
    (void)maxlen;

    return 1;
}

char *RETARGET(_command_string)(char *cmd, int len)
{
    (void)len;

    return cmd;
}

int RETARGET(_isatty)(int fd)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;

    errno = EBADF;
    return 0;
}

int RETARGET(_lseek)(int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;

    errno = EBADF;
    return -1;
}

int RETARGET(_fstat)(int fd, struct stat *st)
{
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return 0;
}

int RETARGET(_kill)(int pid, int sig)
{
    return (-1);
}

int RETARGET(_getpid)()
{
    return (1);
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

int RETARGET(getchar)(void)
{
    return ((int)GetChar());
}

#endif
