/******************************************************************************
 * @file     xmodem.c
 * @version  V1.00
 * @brief    Xmodem transfer
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "xmodem.h"

#define XMD_MAX_TRANS_SIZE      (1024 * 1024)


/* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
static uint8_t s_au8XmdBuf[1030];


/*
    To program data from Xmodem transfer
*/
static int32_t XMD_Write(uint32_t u32Addr, uint32_t u32Data)
{
    uint32_t u32TimeOutCnt;

    FMC->ISPADDR = u32Addr;

    if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0)
    {
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        u32TimeOutCnt = FMC_TIMEOUT_ERASE;

        while (FMC->ISPTRG)
        {
            if (--u32TimeOutCnt == 0)
                return -1;
        }
    }

    FMC->ISPDAT = u32Data;
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    u32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG)
    {
        if (--u32TimeOutCnt == 0)
            return -1;
    }

    return 0;
}


static void XMD_putc(uint8_t c)
{
    UART_T *pUART = XMD_UART_PORT;

    while (pUART->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    pUART->DAT = c;

}

static int32_t XMD_getc()
{
    UART_T *pUART = XMD_UART_PORT;
    uint32_t u32ms = 0;

    /* Wait for 100ms */
    while (u32ms < 100)
    {
        SysTick->CTRL = 0;
        SysTick->LOAD = 1000 * CyclesPerUs; /* 1ms */
        SysTick->VAL  = (0x0UL);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
            if ((pUART->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != UART_FIFOSTS_RXEMPTY_Msk)
            {
                SysTick->CTRL = 0;
                return ((int32_t)pUART->DAT);
            }
        }

        u32ms++;
    }

    SysTick->CTRL = 0;
    return -1; /* time-out */
}

static uint16_t crc16_ccitt(const uint8_t *pu8buf, int32_t i32len)
{
    uint16_t crc = 0;

    while (i32len--)
    {
        int32_t i;
        crc ^= *pu8buf++ << 8;

        for (i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
                crc = (uint16_t)(crc << 1) ^ (uint16_t)0x1021;
            else
                crc = (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static int32_t check(int32_t iscrc, const uint8_t *pu8buf, int32_t i32Size)
{
    if (iscrc)
    {
        uint16_t crc = crc16_ccitt(pu8buf, i32Size);
        uint16_t tcrc = (uint16_t)(pu8buf[i32Size] << 8) + (uint16_t)pu8buf[i32Size + 1];

        if (crc == tcrc)
            return 1;
    }
    else
    {
        int32_t i;
        uint8_t tsum = 0;

        for (i = 0; i < i32Size; ++i)
            tsum += pu8buf[i];

        if (tsum == pu8buf[i32Size])
            return 1;
    }

    return 0;
}




/**
  * @brief      Recive data from UART Xmodem transfer and program the data to flash.
  * @param[in]  u32DestAddr Destination address of flash to program.
  * @return     Recived data size if successful. Return < 0 when error.
  *
  * @details    This function is used to recieve UART data through Xmodem transfer.
  *             The received data will be programmed to flash packet by packet.
  */
int32_t XmodemRecv(uint32_t u32DestAddr)
{
    int32_t i32Err = 0;
    uint8_t *p;
    int32_t bufsz, crc = 0;
    uint8_t trychar = 'C';
    uint8_t packetno = 1;
    int32_t i, j;
    int32_t retrans = MAXRETRANS;
    int32_t i32TransBytes = 0;
    int32_t ch;
    uint32_t u32StarAddr, u32Data;

    for (;;)
    {
        for (i = 0; i < XMD_MAX_TIMEOUT; ++i) /* set timeout period */
        {
            if (trychar)
                XMD_putc(trychar);

            ch = XMD_getc();

            if (ch >= 0)
            {
                switch (ch)
                {
                    case XMD_SOH:
                        bufsz = 128;
                        goto START_RECEIVE;

                    case XMD_STX:
                        bufsz = 1024;
                        goto START_RECEIVE;

                    case XMD_EOT:
                        XMD_putc(XMD_ACK);
                        return (i32Err == 0) ? i32TransBytes : i32Err; /* normal end */

                    case XMD_CAN:
                        XMD_putc(XMD_ACK);
                        return XMD_STS_USER_CANCEL; /* canceled by remote */

                    default:
                        break;
                }
            }
        }

        if (trychar == 'C')
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            return XMD_STS_TIMEOUT; /* too many retry error */
        }

        XMD_putc(XMD_CAN);
        XMD_putc(XMD_CAN);
        XMD_putc(XMD_CAN);
        return XMD_STS_NAK; /* sync error */

START_RECEIVE:

        if (trychar == 'C')
            crc = 1;

        trychar = 0;
        p = s_au8XmdBuf;
        *p++ = (uint8_t)ch;

        for (i = 0; i < (bufsz + (crc ? 1 : 0) + 3); ++i)
        {
            ch = XMD_getc();

            if (ch < 0)
                goto REJECT_RECEIVE;

            *p++ = (char)ch;
        }

        if (s_au8XmdBuf[1] != packetno)
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            return XMD_STS_PACKET_NUM_ERR;
        }
        else
        {
            if (((s_au8XmdBuf[1] + s_au8XmdBuf[2]) == 0xFF) && check(crc, &s_au8XmdBuf[3], bufsz))
            {
                if (s_au8XmdBuf[1] == packetno)
                {
                    volatile int32_t count = XMD_MAX_TRANS_SIZE - i32TransBytes;

                    if (count > bufsz)
                        count = bufsz;

                    if (count > 0)
                    {
                        for (j = 0; j < (bufsz + 3) / 4; j++)
                        {
                            memcpy((uint8_t *)&u32Data, &s_au8XmdBuf[3 + (j * 0x4)], 4);

                            u32StarAddr = u32DestAddr + (uint32_t)i32TransBytes;

                            i32Err = XMD_Write(u32StarAddr + ((uint32_t)j * 0x4), u32Data);

                            if (i32Err < 0)
                                continue;
                        }

                        i32TransBytes += count;
                    }

                    ++packetno;
                    retrans = MAXRETRANS + 1;
                }

                if (--retrans <= 0)
                {
                    XMD_putc(XMD_CAN);
                    XMD_putc(XMD_CAN);
                    XMD_putc(XMD_CAN);
                    return XMD_STS_TIMEOUT; /* too many retry error */
                }

                XMD_putc(XMD_ACK);
                continue;
            }
        }

REJECT_RECEIVE:
        XMD_putc(XMD_NAK);
    }
}

/**
  * @brief      Send data by UART Xmodem transfer.
  * @param[in]  pu8Src       Address of the source data to transfer.
  * @param[in]  i32SrcSize   Size of the total size to transfer.
  * @retval     Total transfer size when successfull
  * @retval     -1  Canceled by remote
  * @retval     -2  No sync chararcter received.
  * @retval     -4  Transmit error.
  * @retval     -5  Unknown error.
  * @details    This function is used to send UART data through Xmodem transfer.
  *
  */
int32_t XmodemSend(uint8_t *pu8Src, int32_t i32SrcSize)
{
    int bufsz, crc = -1;
    unsigned char packetno = 1;
    int i, c, len = 0;
    int retry;

    for (;;)
    {
        for (retry = 0; retry < 160; ++retry)
        {
            if ((c = XMD_getc()) >= 0)
            {
                switch (c)
                {
                    case 'C':
                        crc = 1;
                        goto START_TRANS;

                    case XMD_NAK:
                        crc = 0;
                        goto START_TRANS;

                    case XMD_CAN:
                        if ((c = XMD_getc()) == XMD_CAN)
                        {
                            XMD_putc(XMD_ACK);

                            return -1; /* canceled by remote */
                        }

                        break;

                    default:
                        break;
                }
            }
        }

        if (retry >= 160)
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);

            return -2; /* no sync */
        }

        for (;;)
        {
START_TRANS:
            s_au8XmdBuf[0] = XMD_SOH;
            bufsz = 128;
            s_au8XmdBuf[1] = packetno;
            s_au8XmdBuf[2] = ~packetno;
            c = i32SrcSize - len;

            if (c > bufsz) c = bufsz;

            if (c > 0)
            {
                memset(&s_au8XmdBuf[3], 0, (uint32_t)bufsz);

                if ((c % bufsz) != 0)   /* Pad XMD_CTRLZ if left data is not align with bufsz */
                {
                    s_au8XmdBuf[3] = XMD_CTRLZ;
                }
                else
                {

                    memcpy(&s_au8XmdBuf[3], pu8Src, (uint32_t)c);
                    pu8Src += c;

                    if (c < bufsz) s_au8XmdBuf[3 + c] = XMD_CTRLZ;
                }

                if (crc)
                {
                    unsigned short ccrc = crc16_ccitt(&s_au8XmdBuf[3], bufsz);
                    s_au8XmdBuf[bufsz + 3] = (ccrc >> 8) & 0xFF;
                    s_au8XmdBuf[bufsz + 4] = ccrc & 0xFF;
                }
                else
                {
                    unsigned char ccks = 0;

                    for (i = 3; i < bufsz + 3; ++i)
                    {
                        ccks += s_au8XmdBuf[i];
                    }

                    s_au8XmdBuf[bufsz + 3] = ccks;
                }

                for (retry = 0; retry < MAXRETRANS; ++retry)
                {
                    for (i = 0; i < bufsz + 4 + (crc ? 1 : 0); ++i)
                    {
                        XMD_putc(s_au8XmdBuf[i]);
                    }

                    if ((c = XMD_getc()) >= 0)
                    {
                        switch (c)
                        {
                            case XMD_ACK:
                                ++packetno;
                                len += bufsz;
                                goto START_TRANS;

                            case XMD_CAN:
                                if ((c = XMD_getc()) == XMD_CAN)
                                {
                                    XMD_putc(XMD_ACK);

                                    return -1; /* canceled by remote */
                                }

                                break;

                            case XMD_NAK:
                            default:
                                break;
                        }
                    }
                }

                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                return -4; /* xmit error */
            }
            else
            {
                for (retry = 0; retry < 10; ++retry)
                {
                    XMD_putc(XMD_EOT);

                    if ((c = XMD_getc()) == XMD_ACK) break;
                }

                return (c == XMD_ACK) ? len : -5;
            }
        }
    }
}

