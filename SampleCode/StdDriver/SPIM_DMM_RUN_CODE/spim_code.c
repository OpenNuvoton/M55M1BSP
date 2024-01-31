/**************************************************************************//**
 * @file     spim_code.c
 * @version  V1.00
 * @brief    Collect of sub-routines running on SPIM flash.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

/*
 *  Put a character to DEBUG_PORT transmitter
 */
void spim_putc(int ch)
{
    if ((char)ch == '\n')
    {
        while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}

        DEBUG_PORT->DAT = '\r';
    }

    while (DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) {}

    DEBUG_PORT->DAT = (uint32_t)ch;
}

/*
 *  Poll until received a character from DEBUG_PORT receiver
 */
char spim_getc(void)
{
    while (1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            return ((char)DEBUG_PORT->DAT);
        }
    }
}

/*
 *  print out a string
 */
void spim_put_string(char *str)
{
    while (*str != '\0')
    {
        spim_putc(*str++);
    }
}

void spim_routine(void)
{
    spim_put_string("\r\n\r\n\r\n");
    spim_put_string("+------------------------------------------+\r\n");
    spim_put_string("|  Program is now running on SPIM flash.   |\r\n");
    spim_put_string("+------------------------------------------+\r\n\r\n");

    spim_put_string("\r\nPress any to return to main program on APROM flash...\r\n");

    spim_getc();
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
