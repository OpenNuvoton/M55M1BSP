/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how USB works as a dual role device.
 *           If it works as USB Host, it can access a mass storage device.
 *           If it works as USB Device, it acts as a mass storage device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "usbh_lib.h"
#include "ff.h"
#include "diskio.h"
#include "massstorage.h"

uint8_t g_bIsBdevice = 0, g_bIsAdevice = 0;
extern uint8_t volatile g_u8MscStart;

/* HSOTG interrupt handler */
NVT_ITCM void HSOTG_IRQHandler(void)
{
    __IO uint32_t u32INTSTS, u32INTEN;

    u32INTEN = HSOTG->INTEN;
    u32INTSTS = HSOTG->INTSTS;

    if (u32INTSTS & u32INTEN & HSOTG_INTSTS_IDCHGIF_Msk)
    {
        printf("[ID changed]\n");

        /* Check ID status */
        if (HSOTG_GET_STATUS(HSOTG_STATUS_IDSTS_Msk))
            g_bIsAdevice = 0;

        /* Clear ID status changed interrupt flag */
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_IDCHGIF_Msk);
    }

    if (u32INTSTS & u32INTEN & HSOTG_INTSTS_BVLDCHGIF_Msk)
    {
        printf("[B session valid (HSOTG_STATUS: 0x%x)]\n", HSOTG->STATUS);

        /* Check ID status */
        if (HSOTG_GET_STATUS(HSOTG_STATUS_IDSTS_Msk) == 0)
            g_bIsBdevice = 0;

        /* Clear B-device session valid state change interrupt flag */
        HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_BVLDCHGIF_Msk);
    }

    /* Clear all interrupt flags */
    HSOTG->INTSTS = u32INTSTS;
    /* make sure that interrupt flag has been cleared. */
    u32INTSTS = HSOTG->INTSTS;
}


#define BUFF_SIZE       (2048)

static UINT blen = BUFF_SIZE;
DWORD       acc_size;
WORD        acc_files, acc_dirs;
FILINFO     Finfo;
static FIL  file1, file2;                   /* File objects                               */

char        Line[256];                      /* Console input buffer                       */
#if _USE_LFN
    char        Lfname[512];                    /* Console input buffer                       */
#endif

#ifdef __ICCARM__
    #pragma data_alignment=32
    BYTE        Buff_Pool1[BUFF_SIZE] ;         /* Working buffer 1                           */
    BYTE        Buff_Pool2[BUFF_SIZE] ;         /* Working buffer 2                           */
#else
    BYTE Buff_Pool1[BUFF_SIZE] __attribute__((aligned(32)));    /* Working buffer 1                           */
    BYTE Buff_Pool2[BUFF_SIZE] __attribute__((aligned(32)));    /* Working buffer 2                           */
#endif

BYTE  *Buff1;
BYTE  *Buff2;

volatile uint32_t  g_tick_cnt;

NVT_ITCM void SysTick_Handler(void)
{
    g_tick_cnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    g_tick_cnt = 0;

    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }
}

uint32_t get_ticks()
{
    return g_tick_cnt;
}


/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from HIRC_12M. Prescale 12
     */
    /* TIMER0 clock from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);

    TIMER_SET_PRESCALE_VALUE(TIMER0, (12 - 1));
    /* stop timer0 */
    TIMER_Stop(TIMER0);
    /* write 1 to clear for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* set timer cmp value */
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    /* Timer0 config to oneshot mode */
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* start timer0*/
    TIMER_Start(TIMER0);

    while (TIMER_GetIntFlag(TIMER0) == 0);
}

void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;

    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);

        printf("  ");

        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");

            nBytes--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}


/*----------------------------------------------*/
/* Get a value of the string                    */
/*----------------------------------------------*/
/*  "123 -5   0x3ff 0b1111 0377  w "
        ^                           1st call returns 123 and next ptr
           ^                        2nd call returns -5 and next ptr
                   ^                3rd call returns 1023 and next ptr
                          ^         4th call returns 15 and next ptr
                               ^    5th call returns 255 and next ptr
                                  ^ 6th call fails and returns 0
*/

int xatoi(          /* 0:Failed, 1:Successful */
    TCHAR **str,    /* Pointer to pointer to the string */
    long *res       /* Pointer to a variable to store the value */
)
{
    unsigned long val;
    unsigned char r, s = 0;
    TCHAR c;


    *res = 0;

    while ((c = **str) == ' ')(*str)++;     /* Skip leading spaces */

    if (c == '-')       /* negative? */
    {
        s = 1;
        c = *(++(*str));
    }

    if (c == '0')
    {
        c = *(++(*str));

        switch (c)
        {
            case 'x':       /* hexadecimal */
                r = 16;
                c = *(++(*str));
                break;

            case 'b':       /* binary */
                r = 2;
                c = *(++(*str));
                break;

            default:
                if (c <= ' ') return 1; /* single zero */

                if (c < '0' || c > '9') return 0;   /* invalid char */

                r = 8;      /* octal */
        }
    }
    else
    {
        if (c < '0' || c > '9') return 0;   /* EOL or invalid char */

        r = 10;         /* decimal */
    }

    val = 0;

    while (c > ' ')
    {
        if (c >= 'a') c -= 0x20;

        c -= '0';

        if (c >= 17)
        {
            c -= 7;

            if (c <= 9) return 0;   /* invalid char */
        }

        if (c >= r) return 0;       /* invalid char for current radix */

        val = val * r + c;
        c = *(++(*str));
    }

    if (s) val = 0 - val;           /* apply sign if needed */

    *res = val;
    return 1;
}


/*----------------------------------------------*/
/* Dump a block of byte array                   */

void put_dump(
    const unsigned char *buff,  /* Pointer to the byte array to be dumped */
    unsigned long addr,         /* Heading address value */
    int cnt                     /* Number of bytes to be dumped */
)
{
    int i;


    printf("%08x ", (UINT)addr);

    for (i = 0; i < cnt; i++)
        printf(" %02x", buff[i]);

    printf(" ");

    for (i = 0; i < cnt; i++)
        putchar((TCHAR)((buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.'));

    printf("\n");
}


/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */
/*--------------------------------------------------------------------------*/

static FRESULT scan_files(char *path)
{
    DIR dirs;
    FRESULT res;
    BYTE i;
    char *fn;

    if ((res = f_opendir(&dirs, path)) == FR_OK)
    {
        i = strlen(path);

        while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0])
        {
            if (FF_FS_RPATH && Finfo.fname[0] == '.') continue;

#if _USE_LFN
            fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
            fn = Finfo.fname;
#endif

            if (Finfo.fattrib & AM_DIR)
            {
                acc_dirs++;
                *(path + i) = '/';
                strcpy(path + i + 1, fn);
                res = scan_files(path);
                *(path + i) = '\0';

                if (res != FR_OK) break;
            }
            else
            {
                /*              printf("%s/%s\n", path, fn); */
                acc_files++;
                acc_size += Finfo.fsize;
            }
        }
    }

    return res;
}

void put_rc(FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    //FRESULT i;
    uint32_t i;

    for (i = 0; (i != (UINT)rc) && *p; i++)
    {
        while (*p++) ;
    }

    printf(_T("rc=%d FR_%s\n"), (UINT)rc, p);
}

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

void get_line(char *buff, int len)
{
    TCHAR c;
    int idx = 0;

    for (;;)
    {
        c = getchar();
        putchar(c);

        if (c == '\r') break;

        if ((c == '\b') && idx) idx--;

        if ((c >= ' ') && (idx < len - 1)) buff[idx++] = c;
    }

    buff[idx] = 0;
    putchar('\n');
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

DWORD get_fattime(void)
{
    DWORD tmr;

    tmr = 0x00000;

    return tmr;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Enable GPIOA module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT*/
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

    /* Enable HSUSBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);

    /* Enable HSUSBD module clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Set OTG as USB Host role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x2ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x2UL << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;//Set HSUSB PHY Active.


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PB.10   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PB.11 */
    SET_HSUSB_VBUS_ST_PJ12();


    /* Lock protected registers */
    SYS_LockReg();
}

void USBH_Process()
{
    char        *ptr, *ptr2;
    long        p1, p2, p3;
    BYTE        *buf;
    FATFS       *fs;              /* Pointer to file system object */
    FRESULT     res;
    TCHAR       usb_path[] = { '3', ':', 0 };    /* USB drive started from 3 */

    DIR dir;                /* Directory object */
    UINT s1, s2, cnt;
    static const BYTE ft[] = {0, 12, 16, 32};
    DWORD ofs = 0, sect = 0;
    uint32_t    t0;

    usbh_pooling_hubs();
    f_chdrive(usb_path);          /* set default path */

    for (;;)
    {

        if (!g_bIsAdevice)
        {
            printf("break-A (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
            return;
        }

        usbh_pooling_hubs();

        printf(_T(">"));
        ptr = Line;

        get_line(ptr, sizeof(Line));

        switch (*ptr++)
        {

            case '4' :
            case '5' :
            case '6' :
            case '7' :
                ptr--;
                *(ptr + 1) = ':';
                *(ptr + 2) = 0;
                put_rc(f_chdrive((TCHAR *)ptr));
                break;

            case 'd' :     /* d [<lba>] - Dump sector */
                if (!xatoi(&ptr, &p2)) p2 = sect;

                res = (FRESULT)disk_read(3, Buff1, p2, 1);

                if (res)
                {
                    printf("rc=%d\n", (WORD)res);
                    break;
                }

                sect = p2 + 1;
                printf("Sector:%d\n", (INT)p2);

                for (buf = (unsigned char *)Buff1, ofs = 0; ofs < 0x200; buf += 16, ofs += 16)
                    put_dump(buf, ofs, 16);

                break;

            case 'b' :
                switch (*ptr++)
                {
                    case 'd' :  /* bd <addr> - Dump R/W buffer */
                        if (!xatoi(&ptr, &p1)) break;

                        for (ptr = (char *)&Buff1[p1], ofs = p1, cnt = 32; cnt; cnt--, ptr += 16, ofs += 16)
                            put_dump((BYTE *)ptr, ofs, 16);

                        break;

                    case 'e' :  /* be <addr> [<data>] ... - Edit R/W buffer */
                        if (!xatoi(&ptr, &p1)) break;

                        if (xatoi(&ptr, &p2))
                        {
                            do
                            {
                                Buff1[p1++] = (BYTE)p2;
                            } while (xatoi(&ptr, &p2));

                            break;
                        }

                        for (;;)
                        {
                            printf("%04X %02X-", (WORD)p1, Buff1[p1]);
                            get_line(Line, sizeof(Line));
                            ptr = Line;

                            if (*ptr == '.') break;

                            if (*ptr < ' ')
                            {
                                p1++;
                                continue;
                            }

                            if (xatoi(&ptr, &p2))
                                Buff1[p1++] = (BYTE)p2;
                            else
                                printf("???\n");
                        }

                        break;

                    case 'r' :  /* br <sector> [<n>] - Read disk into R/W buffer */
                        if (!xatoi(&ptr, &p2)) break;

                        if (!xatoi(&ptr, &p3)) p3 = 1;

                        printf("rc=%d\n", disk_read(0, Buff1, p2, p3));
                        break;

                    case 'w' :  /* bw <sector> [<n>] - Write R/W buffer into disk */
                        if (!xatoi(&ptr, &p2)) break;

                        if (!xatoi(&ptr, &p3)) p3 = 1;

                        printf("rc=%d\n", disk_write(0, Buff1, p2, p3));
                        break;

                    case 'f' :  /* bf <n> - Fill working buffer */
                        if (!xatoi(&ptr, &p1)) break;

                        memset(Buff1, (int)p1, BUFF_SIZE);
                        break;

                }

                break;

            case 'f' :
                switch (*ptr++)
                {

                    case 's' :  /* fs - Show logical drive status */
                        res = f_getfree("", (DWORD *)&p2, &fs);

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        printf("FAT type = FAT%d\nBytes/Cluster = %d\nNumber of FATs = %d\n"
                               "Root DIR entries = %d\nSectors/FAT = %d\nNumber of clusters = %d\n"
                               "FAT start (lba) = %d\nDIR start (lba,cluster) = %d\nData start (lba) = %d\n\n...",
                               ft[fs->fs_type & 3], (INT)(fs->csize * 512UL), fs->n_fats,
                               fs->n_rootdir, (INT)fs->fsize, (INT)(fs->n_fatent - 2),
                               (INT)fs->fatbase, (INT)fs->dirbase, (INT)fs->database
                              );
                        acc_size = acc_files = acc_dirs = 0;
#if _USE_LFN
                        Finfo.lfname = Lfname;
                        Finfo.lfsize = sizeof(Lfname);
#endif
                        res = scan_files(ptr);

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        printf("\r%d files, %d bytes.\n%d folders.\n"
                               "%d KB total disk space.\n%d KB available.\n",
                               acc_files, (INT)acc_size, acc_dirs,
                               (INT)(fs->n_fatent - 2) * (INT)(fs->csize / 2), (INT)(p2 * (fs->csize / 2))
                              );
                        break;

                    case 'l' :  /* fl [<path>] - Directory listing */
                        while (*ptr == ' ') ptr++;

                        res = f_opendir(&dir, ptr);

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        p1 = s1 = s2 = 0;

                        for (;;)
                        {
                            res = f_readdir(&dir, &Finfo);

                            if ((res != FR_OK) || !Finfo.fname[0]) break;

                            if (Finfo.fattrib & AM_DIR)
                            {
                                s2++;
                            }
                            else
                            {
                                s1++;
                                p1 += Finfo.fsize;
                            }

                            printf("%c%c%c%c%c %d/%02d/%02d %02d:%02d    %9d  %s",
                                   (Finfo.fattrib & AM_DIR) ? 'D' : '-',
                                   (Finfo.fattrib & AM_RDO) ? 'R' : '-',
                                   (Finfo.fattrib & AM_HID) ? 'H' : '-',
                                   (Finfo.fattrib & AM_SYS) ? 'S' : '-',
                                   (Finfo.fattrib & AM_ARC) ? 'A' : '-',
                                   (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
                                   (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, (INT)Finfo.fsize, Finfo.fname);
#if _USE_LFN

                            for (p2 = strlen(Finfo.fname); p2 < 14; p2++)
                                printf(" ");

                            printf("%s\n", Lfname);
#else
                            printf("\n");
#endif
                        }

                        printf("%4d File(s),%10d bytes total\n%4d Dir(s)", s1, (INT)p1, s2);

                        if (f_getfree(ptr, (DWORD *)&p1, &fs) == FR_OK)
                            printf(", %10d bytes free\n", (INT)p1 * fs->csize * 512);

                        break;


                    case 'o' :  /* fo <mode> <file> - Open a file */
                        if (!xatoi(&ptr, &p1)) break;

                        while (*ptr == ' ') ptr++;

                        put_rc(f_open(&file1, ptr, (BYTE)p1));
                        break;

                    case 'c' :  /* fc - Close a file */
                        put_rc(f_close(&file1));
                        break;

                    case 'e' :  /* fe - Seek file pointer */
                        if (!xatoi(&ptr, &p1)) break;

                        res = f_lseek(&file1, p1);
                        put_rc(res);

                        if (res == FR_OK)
                            printf("fptr=%d(0x%X)\n", (INT)file1.fptr, file1.fptr);

                        break;

                    case 'd' :  /* fd <len> - read and dump file from current fp */
                        if (!xatoi(&ptr, &p1)) break;

                        ofs = file1.fptr;

                        while (p1)
                        {
                            if ((UINT)p1 >= 16)
                            {
                                cnt = 16;
                                p1 -= 16;
                            }
                            else
                            {
                                cnt = p1;
                                p1 = 0;
                            }

                            res = f_read(&file1, Buff1, cnt, &cnt);

                            if (res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }

                            if (!cnt) break;

                            put_dump(Buff1, ofs, cnt);
                            ofs += 16;
                        }

                        break;

                    case 'r' :  /* fr <len> - read file */
                        if (!xatoi(&ptr, &p1)) break;

                        p2 = 0;
                        t0 = get_ticks();

                        while (p1)
                        {
                            if ((UINT)p1 >= blen)
                            {
                                cnt = blen;
                                p1 -= blen;
                            }
                            else
                            {
                                cnt = p1;
                                p1 = 0;
                            }

                            res = f_read(&file1, Buff1, cnt, &s2);

                            if (res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }

                            p2 += s2;

                            if (cnt != s2) break;
                        }

                        p1 = get_ticks() - t0;

                        if (p1)
                            printf("%d bytes read with %d kB/sec.\n", (INT)p2, (INT)(((p2 * 100) / p1) / 1024));

                        break;

                    case 'w' :  /* fw <len> <val> - write file */
                        if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;

                        memset(Buff1, (BYTE)p2, blen);
                        p2 = 0;
                        t0 = get_ticks();

                        while (p1)
                        {
                            if ((UINT)p1 >= blen)
                            {
                                cnt = blen;
                                p1 -= blen;
                            }
                            else
                            {
                                cnt = p1;
                                p1 = 0;
                            }

                            res = f_write(&file1, Buff1, cnt, &s2);

                            if (res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }

                            p2 += s2;

                            if (cnt != s2) break;
                        }

                        p1 = get_ticks() - t0;

                        if (p1)
                            printf("%d bytes written with %d kB/sec.\n", (INT)p2, (INT)(((p2 * 100) / p1) / 1024));

                        break;

                    case 'n' :  /* fn <old_name> <new_name> - Change file/dir name */
                        while (*ptr == ' ') ptr++;

                        ptr2 = strchr(ptr, ' ');

                        if (!ptr2) break;

                        *ptr2++ = 0;

                        while (*ptr2 == ' ') ptr2++;

                        put_rc(f_rename(ptr, ptr2));
                        break;

                    case 'u' :  /* fu <name> - Unlink a file or dir */
                        while (*ptr == ' ') ptr++;

                        put_rc(f_unlink(ptr));
                        break;

                    case 'v' :  /* fv - Truncate file */
                        put_rc(f_truncate(&file1));
                        break;

                    case 'k' :  /* fk <name> - Create a directory */
                        while (*ptr == ' ') ptr++;

                        put_rc(f_mkdir(ptr));
                        break;

                    case 'a' :  /* fa <atrr> <mask> <name> - Change file/dir attribute */
                        if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;

                        while (*ptr == ' ') ptr++;

                        put_rc(f_chmod(ptr, p1, p2));
                        break;

                    case 't' :  /* ft <year> <month> <day> <hour> <min> <sec> <name> - Change time stamp */
                        if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;

                        Finfo.fdate = (WORD)(((p1 - 1980) << 9) | ((p2 & 15) << 5) | (p3 & 31));

                        if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;

                        Finfo.ftime = (WORD)(((p1 & 31) << 11) | ((p1 & 63) << 5) | ((p1 >> 1) & 31));
                        put_rc(f_utime(ptr, &Finfo));
                        break;

                    case 'x' : /* fx <src_name> <dst_name> - Copy file */
                        while (*ptr == ' ') ptr++;

                        ptr2 = strchr(ptr, ' ');

                        if (!ptr2) break;

                        *ptr2++ = 0;

                        while (*ptr2 == ' ') ptr2++;

                        printf("Opening \"%s\"", ptr);
                        res = f_open(&file1, ptr, FA_OPEN_EXISTING | FA_READ);
                        printf("\n");

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        printf("Creating \"%s\"", ptr2);
                        res = f_open(&file2, ptr2, FA_CREATE_ALWAYS | FA_WRITE);
                        putchar('\n');

                        if (res)
                        {
                            put_rc(res);
                            f_close(&file1);
                            break;
                        }

                        printf("Copying...");
                        p1 = 0;

                        for (;;)
                        {
                            res = f_read(&file1, Buff1, BUFF_SIZE, &s1);

                            if (res || s1 == 0) break;   /* error or eof */

                            res = f_write(&file2, Buff1, s1, &s2);
                            p1 += s2;

                            if (res || s2 < s1) break;   /* error or disk full */
                        }

                        printf("\n%d bytes copied.\n", (INT)p1);
                        f_close(&file1);
                        f_close(&file2);
                        break;

                    case 'y' : /* fz <src_name> <dst_name> - Compare file */
                        while (*ptr == ' ') ptr++;

                        ptr2 = strchr(ptr, ' ');

                        if (!ptr2) break;

                        *ptr2++ = 0;

                        while (*ptr2 == ' ') ptr2++;

                        printf("Opening \"%s\"", ptr);
                        res = f_open(&file1, ptr, FA_OPEN_EXISTING | FA_READ);
                        printf("\n");

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        printf("Opening \"%s\"", ptr2);
                        res = f_open(&file2, ptr2, FA_OPEN_EXISTING | FA_READ);
                        putchar('\n');

                        if (res)
                        {
                            put_rc(res);
                            f_close(&file1);
                            break;
                        }

                        printf("Compare...");
                        p1 = 0;

                        for (;;)
                        {
                            res = f_read(&file1, Buff1, BUFF_SIZE, &s1);

                            if (res || s1 == 0)
                            {
                                printf("\nRead file %s terminated. (%d)\n", ptr, res);
                                break;     /* error or eof */
                            }

                            res = f_read(&file2, Buff2, BUFF_SIZE, &s2);

                            if (res || s2 == 0)
                            {
                                printf("\nRead file %s terminated. (%d)\n", ptr2, res);
                                break;     /* error or eof */
                            }

                            p1 += s2;

                            if (res || s2 < s1) break;   /* error or disk full */

                            if (memcmp(Buff1, Buff2, s1) != 0)
                            {
                                printf("Compare failed!!\n");
                                break;
                            }

                            if ((p1 % 0x10000) == 0)
                                printf("\n%d KB compared.", (INT)(p1 / 1024));

                            printf(".");
                        }

                        if (s1 == 0)
                            printf("\nPASS. \n ");

                        f_close(&file1);
                        f_close(&file2);
                        break;

#if _FS_RPATH

                    case 'g' :  /* fg <path> - Change current directory */
                        while (*ptr == ' ') ptr++;

                        put_rc(f_chdir(ptr));
                        break;

                    case 'j' :  /* fj <drive#> - Change current drive */
                        while (*ptr == ' ') ptr++;

                        dump_buff_hex((uint8_t *)&p1, 16);
                        put_rc(f_chdrive((TCHAR *)ptr));
                        break;
#endif
#if _USE_MKFS

                    case 'm' :  /* fm <partition rule> <sect/clust> - Create file system */
                        if (!xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;

                        printf("The memory card will be formatted. Are you sure? (Y/n)=");
                        get_line(ptr, sizeof(Line));

                        if (*ptr == 'Y')
                            put_rc(f_mkfs(0, (BYTE)p2, (WORD)p3));

                        break;
#endif

                    case 'z' :  /* fz [<rw size>] - Change R/W length for fr/fw/fx command */
                        if (xatoi(&ptr, &p1) && p1 >= 1 && (size_t)p1 <= BUFF_SIZE)
                            blen = p1;

                        printf("blen=%d\n", blen);
                        break;
                }

                break;

            case '?':       /* Show usage */
                printf(
                    _T("n: - Change default drive (USB drive is 3~7)\n")
                    _T("d [<lba>] - Dump sector\n")
                    //_T("ds <pd#> - Show disk status\n")
                    _T("\n")
                    _T("bd <ofs> - Dump working buffer\n")
                    _T("be <ofs> [<data>] ... - Edit working buffer\n")
                    _T("br <pd#> <sect> [<num>] - Read disk into working buffer\n")
                    _T("bw <pd#> <sect> [<num>] - Write working buffer into disk\n")
                    _T("bf <val> - Fill working buffer\n")
                    _T("\n")
                    _T("fs - Show volume status\n")
                    _T("fl [<path>] - Show a directory\n")
                    _T("fo <mode> <file> - Open a file\n")
                    _T("fc - Close the file\n")
                    _T("fe <ofs> - Move fp in normal seek\n")
                    //_T("fE <ofs> - Move fp in fast seek or Create link table\n")
                    _T("fd <len> - Read and dump the file\n")
                    _T("fr <len> - Read the file\n")
                    _T("fw <len> <val> - Write to the file\n")
                    _T("fn <object name> <new name> - Rename an object\n")
                    _T("fu <object name> - Unlink an object\n")
                    _T("fv - Truncate the file at current fp\n")
                    _T("fk <dir name> - Create a directory\n")
                    _T("fa <atrr> <mask> <object name> - Change object attribute\n")
                    _T("ft <year> <month> <day> <hour> <min> <sec> <object name> - Change timestamp of an object\n")
                    _T("fx <src file> <dst file> - Copy a file\n")
                    _T("fg <path> - Change current directory\n")
                    _T("fj <ld#> - Change current drive. For example: <fj 4:>\n")
                    _T("fm <ld#> <rule> <cluster size> - Create file system\n")
                    _T("\n")
                );
                break;
        }
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    enable_sys_tick(100);
    printf("\n\n");
    printf("system clock :%uMHz\r\n", SystemCoreClock);
    printf("+------------------------------------------------------+\n");
    printf("|                                                      |\n");
    printf("|    HSOTG ID dependent Mass Storage sample program    |\n");
    printf("|                                                      |\n");
    printf("+------------------------------------------------------+\n");

    printf("The USB role is determined by the ID pin state of USB cable.\n");
    printf("If acts as a USB host, it can access a mass storage device with a simple file system. ");
    printf("Type '?' on command line to show all supported commands.\n");
    printf("If attempts to switch the USB role as a USB device, press 'Enter' key after unplugging the USB cable to exit the file system. ");
    printf("Then plug a proper USB connector to switch the USB role as a mass storage device. The internal data flash is used as the storage.\n");

    HSOTG_ENABLE_PHY();
    /* Enable ID detection function */
    HSOTG_ENABLE_ID_DETECT();
    NVIC_EnableIRQ(HSOTG_IRQn);
    delay_us(1000);

    Buff1 = (BYTE *)((uint32_t)&Buff_Pool1[0]);
    Buff2 = (BYTE *)((uint32_t)&Buff_Pool2[0]);

    usbh_core_init();
    usbh_umas_init();

    while (1)
    {
        if (HSOTG_GET_STATUS(HSOTG_STATUS_IDSTS_Msk))  /* B-device */
        {
            if (HSOTG_GET_STATUS(HSOTG_STATUS_BVLD_Msk))  /* plug-in */
            {
                g_bIsBdevice = 1;
                printf("B-device (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
                HSUSBD_Open(&gsHSInfo, MSC_ClassRequest, NULL);
                MSC_Init();
                NVIC_EnableIRQ(HSUSBD_IRQn);

                /* Clear B-device session valid state change interrupt flag */
                HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_BVLDCHGIF_Msk);
                /* Enable B-device session valid state change interrupt */
                HSOTG_ENABLE_INT(HSOTG_INTEN_BVLDCHGIEN_Msk);

                while (1)
                {
                    if (HSOTG_GET_STATUS(HSOTG_STATUS_BVLD_Msk) == 0)
                        break;

                    if (g_u8MscStart)
                        MSC_ProcessCmd();
                }

                /* Disable B-device session valid state change interrupt */
                HSOTG->INTEN &= ~HSOTG_INTEN_BVLDCHGIEN_Msk;
                /* Clear B-device session valid state change interrupt flag */
                HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_BVLDCHGIF_Msk);
                printf("break-B (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
            }
        }
        else     /* A-device */
        {
            g_bIsAdevice = 1;
            printf("A-device (HSOTG_STATUS: 0x%x)\n", HSOTG->STATUS);
            /* Clear ID status changed interrupt flag */
            HSOTG_CLR_INT_FLAG(HSOTG_INTSTS_IDCHGIF_Msk);
            /* Enable ID status changed interrupt */
            HSOTG_ENABLE_INT(HSOTG_INTEN_IDCHGIEN_Msk);
            USBH_Process();
            /* Disable ID status changed interrupt */
            HSOTG_DISABLE_INT(HSOTG_INTEN_IDCHGIEN_Msk);
        }
    }
}
