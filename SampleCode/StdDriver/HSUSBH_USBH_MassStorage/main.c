/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample uses a command-shell-like interface to demonstrate
 *           how to use USBH mass storage driver and make it working as a disk
 *           driver under FATFS file system.
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "diskio.h"
#include "ff.h"
#include "usbh_lib.h"

#define USE_USB_APLL1_CLOCK         1
#define BUFF_SIZE                   (4*1024)

static UINT blen = BUFF_SIZE;
DWORD acc_size;                         /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;

char Line[256];                         /* Console input buffer */
#if _USE_LFN
    char Lfname[512];
#endif

#ifdef __ICCARM__
    #pragma data_alignment=32
    BYTE Buff_Pool[BUFF_SIZE] ;       /* Working buffer */
    BYTE Buff_Pool2[BUFF_SIZE] ;      /* Working buffer 2 */
#else
    BYTE Buff_Pool[BUFF_SIZE] __attribute__((aligned(32)));       /* Working buffer */
    BYTE Buff_Pool2[BUFF_SIZE] __attribute__((aligned(32)));      /* Working buffer 2 */
#endif

BYTE  *Buff;
BYTE  *Buff2;

#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t  buff1[BUFF_SIZE] ;       /* Working buffer */
    uint8_t  buff2[BUFF_SIZE] ;       /* Working buffer */
#else
    uint8_t  buff1[BUFF_SIZE] __attribute__((aligned(4)));       /* Working buffer */
    uint8_t  buff2[BUFF_SIZE] __attribute__((aligned(4)));       /* Working buffer */
#endif

int kbhit(void)
{
    return !((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0);
}

volatile int  int_in_cnt, int_out_cnt, iso_in_cnt, iso_out_cnt;

volatile uint32_t  g_tick_cnt;
uint32_t           g_t0;

volatile int  int_cnt = 0;

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

void timer_init()
{
    g_t0 = get_ticks();
}

uint32_t get_timer_value()
{
    return (get_ticks() - g_t0);
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

        for (i = 0; (i < 16) && (nBytes > 0); i++)
        {
            printf("%02x ", pucBuff[nIdx + i]);
            nBytes--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}

/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */

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

static
FRESULT scan_files(
    char *path      /* Pointer to the path name working buffer */
)
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
    //  DWORD dw;


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


static FIL file1, file2;        /* File objects */


/*
 *  Interrupt-In transfer data delivery callback function.
 */
void int_xfer_read(uint8_t *data_buff, int *data_len)
{
    //  Receive interrupt in transfer data. The data len is (*data_len).
    //  NOTICE: This callback function is in USB Host interrupt context!

    int_in_cnt++;
}


/*
 *  Interrupt-Out transfer data request callback function.
 */
void int_xfer_write(uint8_t *data_buff, int *data_len)
{
    //  LBK request the interrupt out transfer data.
    //  NOTICE: This callback function is in USB Host interrupt context!
    int_out_cnt++;
    *data_len = 8;
    memset(data_buff, int_out_cnt & 0xff, 8);
}

/*
 *  Isochronous-Out transfer data request callback function.
 */
void iso_xfer_write(uint8_t *data_buff, int data_len)
{
    //  Application feeds Isochronous-Out data here.
    //  NOTICE: This callback function is in USB Host interrupt context!
    iso_out_cnt++;
    memset(data_buff, iso_out_cnt & 0xff, data_len);
}

/*
 *  Isochronous-In transfer data request callback function.
 */
void iso_xfer_read(uint8_t *data_buff, int data_len)
{
    //  Application gets Isochronous-In data here.
    //  NOTICE: This callback function is in USB Host interrupt context!
    iso_in_cnt++;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

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

#if (USE_USB_APLL1_CLOCK)
    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);
#endif

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

#if (USE_USB_APLL1_CLOCK)
    /* USB Host desired input clock is 48 MHz. Set as APLL1 divided by 2 (96/2 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));
#else
    /* USB Host desired input clock is 48 MHz. Set as HIRC48M divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH0_MODULE);
    CLK_EnableModuleClock(USBD0_MODULE);
    CLK_EnableModuleClock(OTG0_MODULE);
    /* Enable HSUSBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);

    /* Set OTG as USB Host role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x1ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x1 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;
    //delay_us(20);

    /* Set Debug Uart CLK*/
    SetDebugUartCLK();
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SET_USB_VBUS_ST_PB14();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    char        *ptr, *ptr2;
    long        p1, p2, p3;
    BYTE        *buf;
    FATFS       *fs;              /* Pointer to file system object */
    TCHAR       usb_path[] = { '3', ':', 0 };    /* USB drive started from 3 */
    FRESULT     res;

    DIR dir;                /* Directory object */
    UINT s1, s2, cnt, sector_no;
    static const BYTE ft[] = {0, 12, 16, 32};
    DWORD ofs = 0, sect = 0;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    InitDebugUart();                   /* Init DeubgUART for printf */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+-----------------------------------------------+\n");
    printf("|                                               |\n");
    printf("|     USB Host Mass Storage sample program      |\n");
    printf("|                                               |\n");
    printf("+-----------------------------------------------+\n");

    Buff = (BYTE *)((uint32_t)&Buff_Pool[0]);
    Buff2 = (BYTE *)((uint32_t)&Buff_Pool2[0]);

    usbh_core_init();
    usbh_umas_init();

    usbh_pooling_hubs();

    printf("Init time: %d\n", get_ticks());

    int_in_cnt = int_out_cnt = iso_in_cnt = iso_out_cnt = 0;

    f_chdrive(usb_path);               /* set default path                                */

    for (;;)
    {

        usbh_pooling_hubs();

        usbh_memory_used();            /* print out UsbHostLib memory usage information   */

        printf(_T(">"));
        ptr = Line;

        get_line(ptr, sizeof(Line));

        switch (*ptr++)
        {

            case 'q' :  /* Exit program */
                return 0;

            case '5':
                for (sector_no = 0; sector_no < 128000; sector_no++)
                {
                    if (sector_no % 1000 == 0)
                        printf("\nLBK transfer count: Ii:%d, Io: %d, Si:%d, So:%d\n", int_in_cnt, int_out_cnt, iso_in_cnt, iso_out_cnt);

                    printf("Test sector %d\r", sector_no);

                    memset(buff1, 87, 512);
                    memset(buff2, 0x87, 512);

                    res = (FRESULT)disk_read(3, buff1, sector_no, 1);

                    if (res)
                    {
                        printf("read failed at %d, rc=%d\n", sector_no, (WORD)res);
                        put_rc(res);
                        break;
                    }

                    res = (FRESULT)disk_read(3, buff2, sector_no, 1);

                    if (res)
                    {
                        printf("read failed at %d, rc=%d\n", sector_no, (WORD)res);
                        put_rc(res);
                        break;
                    }

                    if (memcmp(buff1, buff2, 512) != 0)
                    {
                        printf("\nData compare failed!!\n");
                        break;
                    }
                }

                break;

            case '4' :
            case '6' :
            case '7' :
                ptr--;
                *(ptr + 1) = ':';
                *(ptr + 2) = 0;
                put_rc(f_chdrive((TCHAR *)ptr));
                break;

            case 'd' :
                switch (*ptr++)
                {
                    case 'd' :  /* dd [<lba>] - Dump sector */
                        if (!xatoi(&ptr, &p2)) p2 = sect;

                        res = (FRESULT)disk_read(3, Buff, p2, 1);

                        if (res)
                        {
                            printf("rc=%d\n", (WORD)res);
                            break;
                        }

                        sect = p2 + 1;
                        printf("Sector:%d\n", (INT)p2);

                        for (buf = (unsigned char *)Buff, ofs = 0; ofs < 0x200; buf += 16, ofs += 16)
                            put_dump(buf, ofs, 16);

                        break;

                    case 't' :  /* dt - raw sector read/write performance test */
                        printf("Raw sector read performance test...\n");
                        timer_init();

                        for (s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            s2 = BUFF_SIZE / 512;   /* sector count for each read  */
                            res = (FRESULT)disk_read(3, Buff, 10000 + s1 * s2, s2);

                            if (res)
                            {
                                printf("read failed at %d, rc=%d\n", 10000 + s1 * s2, (WORD)res);
                                put_rc(res);
                                break;
                            }

                            if (s1 % 256 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }

                        p1 = get_timer_value();
                        printf("time = %d.%02d\n", (INT)(p1 / 100), (INT)(p1 % 100));
                        printf("Raw read speed: %d KB/s\n", (INT)(((0x800000 * 100) / p1) / 1024));

                        printf("Raw sector write performance test...\n");
                        timer_init();

                        for (s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            s2 = BUFF_SIZE / 512;   /* sector count for each read  */
                            res = (FRESULT)disk_write(3, Buff, 20000 + s1 * s2, s2);

                            if (res)
                            {
                                printf("write failed at %d, rc=%d\n", 20000 + s1 * s2, (WORD)res);
                                put_rc(res);
                                break;
                            }

                            if (s1 % 256 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }

                        p1 = get_timer_value();
                        printf("time = %d.%02d\n", (INT)(p1 / 100), (INT)(p1 % 100));
                        printf("Raw write speed: %d KB/s\n", (INT)(((0x800000 * 100) / p1) / 1024));
                        break;

                    case 'z' :  /* dz - file read/write performance test */
#if 1
                        printf("File write performance test...\n");
                        res = f_open(&file1, "3:\\tfile", FA_CREATE_ALWAYS | FA_WRITE);

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        timer_init();

                        for (s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            res = f_write(&file1, Buff, BUFF_SIZE, &cnt);

                            if (res || (cnt != BUFF_SIZE))
                            {
                                put_rc(res);
                                break;
                            }

                            if (s1 % 128 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }

                        p1 = get_timer_value();
                        f_close(&file1);
                        printf("time = %d.%02d\n", (INT)p1 / 100, (INT)p1 % 100);
                        printf("File write speed: %d KB/s\n", (INT)((0x800000 * 100) / p1) / 1024);
#endif
                        printf("File read performance test...\n");
                        res = f_open(&file1, "3:\\tfile", FA_READ);

                        if (res)
                        {
                            put_rc(res);
                            break;
                        }

                        timer_init();

                        for (s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            res = f_read(&file1, Buff, BUFF_SIZE, &cnt);

                            if (res || (cnt != BUFF_SIZE))
                            {
                                put_rc(res);
                                break;
                            }

                            if (s1 % 128 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }

                        p1 = get_timer_value();
                        f_close(&file1);
                        printf("time = %d.%02d\n", (INT)(p1 / 100), (INT)(p1 % 100));
                        printf("File read speed: %d KB/s\n", (INT)(((0x800000 * 100) / p1) / 1024));
                        break;
                }

                break;

            case 'b' :
                switch (*ptr++)
                {
                    case 'd' :  /* bd <addr> - Dump R/W buffer */
                        if (!xatoi(&ptr, &p1)) break;

                        for (ptr = (char *)&Buff[p1], ofs = p1, cnt = 32; cnt; cnt--, ptr += 16, ofs += 16)
                            put_dump((BYTE *)ptr, ofs, 16);

                        break;

                    case 'e' :  /* be <addr> [<data>] ... - Edit R/W buffer */
                        if (!xatoi(&ptr, &p1)) break;

                        if (xatoi(&ptr, &p2))
                        {
                            do
                            {
                                Buff[p1++] = (BYTE)p2;
                            } while (xatoi(&ptr, &p2));

                            break;
                        }

                        for (;;)
                        {
                            printf("%04X %02X-", (WORD)p1, Buff[p1]);
                            get_line(Line, sizeof(Line));
                            ptr = Line;

                            if (*ptr == '.') break;

                            if (*ptr < ' ')
                            {
                                p1++;
                                continue;
                            }

                            if (xatoi(&ptr, &p2))
                                Buff[p1++] = (BYTE)p2;
                            else
                                printf("???\n");
                        }

                        break;

                    case 'r' :  /* br <sector> [<n>] - Read disk into R/W buffer */
                        if (!xatoi(&ptr, &p2)) break;

                        if (!xatoi(&ptr, &p3)) p3 = 1;

                        printf("rc=%d\n", disk_read(3, Buff, p2, p3));
                        break;

                    case 'w' :  /* bw <sector> [<n>] - Write R/W buffer into disk */
                        if (!xatoi(&ptr, &p2)) break;

                        if (!xatoi(&ptr, &p3)) p3 = 1;

                        printf("rc=%d\n", disk_write(3, Buff, p2, p3));
                        break;

                    case 'f' :  /* bf <n> - Fill working buffer */
                        if (!xatoi(&ptr, &p1)) break;

                        memset(Buff, (int)p1, BUFF_SIZE);
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
                               "FAT start (lba) = %d\nDIR start (lba,clustor) = %d\nData start (lba) = %d\n\n...",
                               ft[fs->fs_type & 3], (INT)(fs->csize * 512UL), fs->n_fats,
                               fs->n_rootdir, (INT)(fs->fsize), (INT)(fs->n_fatent - 2),
                               (INT)(fs->fatbase), (INT)(fs->dirbase), (INT)(fs->database)
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
                               acc_files, (INT)(acc_size), acc_dirs,
                               (INT)((fs->n_fatent - 2) * (fs->csize / 2)), (INT)(p2 * (fs->csize / 2))
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
                            printf(", %10d bytes free\n", (INT)(p1 * fs->csize * 512));

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

                            res = f_read(&file1, Buff, cnt, &cnt);

                            if (res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }

                            if (!cnt) break;

                            put_dump(Buff, ofs, cnt);
                            ofs += 16;
                        }

                        break;

                    case 'r' :  /* fr <len> - read file */
                        if (!xatoi(&ptr, &p1)) break;

                        p2 = 0;
                        timer_init();

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

                            res = f_read(&file1, Buff, cnt, &s2);

                            if (res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }

                            p2 += s2;

                            if (cnt != s2) break;
                        }

                        p1 = get_timer_value();

                        if (p1)
                            printf("%d bytes read with %d kB/sec.\n", (INT)p2, (INT)(((p2 * 100) / p1) / 1024));

                        break;

                    case 'w' :  /* fw <len> <val> - write file */
                        if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;

                        memset(Buff, (BYTE)p2, blen);

                        p2 = 0;
                        timer_init();

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

                            res = f_write(&file1, Buff, cnt, &s2);

                            if (res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }

                            p2 += s2;

                            if (cnt != s2) break;
                        }

                        p1 = get_timer_value();

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

                    case 't' :  /* ft <year> <month> <day> <hour> <min> <sec> <name> - Change timestamp */
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
                            res = f_read(&file1, Buff, BUFF_SIZE, &s1);

                            if (res || s1 == 0) break;   /* error or eof */

                            res = f_write(&file2, Buff, s1, &s2);
                            p1 += s2;

                            if (res || s2 < s1) break;   /* error or disk full */

                            if ((p1 % 0x10000) == 0)
                                printf("\n%d KB copyed.", (INT)(p1 / 1024));

                            printf(".");
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
                            res = f_read(&file1, Buff, BUFF_SIZE, &s1);

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

                            if (memcmp(Buff, Buff2, s1) != 0)
                            {
                                printf("Compre failed!! %d %d\n", s1, s2);
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
                    _T("dd [<lba>] - Dump sector\n")
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

    //while(1);
}


/*** (C) 2023 Nuvoton Technology Corp. All rights reserved. ***/
